/*	$NetBSD: radixtree.c,v 1.27 2020/05/14 08:34:19 msaitoh Exp $	*/

/*-
 * Copyright (c)2011,2012,2013 YAMAMOTO Takashi,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * radixtree.c
 *
 * Overview:
 *
 * This is an implementation of radix tree, whose keys are uint64_t and leafs
 * are user provided pointers.
 *
 * Leaf nodes are just void * and this implementation doesn't care about
 * what they actually point to.  However, this implementation has an assumption
 * about their alignment.  Specifically, this implementation assumes that their
 * 2 LSBs are always zero and uses them for internal accounting.
 *
 * Intermediate nodes and memory allocation:
 *
 * Intermediate nodes are automatically allocated and freed internally and
 * basically users don't need to care about them.  The allocation is done via
 * pool_cache_get(9) for _KERNEL, malloc(3) for userland, and alloc() for
 * _STANDALONE environment.  Only radix_tree_insert_node function can allocate
 * memory for intermediate nodes and thus can fail for ENOMEM.
 *
 * Memory Efficiency:
 *
 * It's designed to work efficiently with dense index distribution.
 * The memory consumption (number of necessary intermediate nodes) heavily
 * depends on the index distribution.  Basically, more dense index distribution
 * consumes less nodes per item.  Approximately,
 *
 *  - the best case: about RADIX_TREE_PTR_PER_NODE items per intermediate node.
 *    it would look like the following.
 *
 *     root (t_height=1)
 *      |
 *      v
 *      [ | | | ]   (intermediate node.  RADIX_TREE_PTR_PER_NODE=4 in this fig)
 *       | | | |
 *       v v v v
 *       p p p p    (items)
 *
 *  - the worst case: RADIX_TREE_MAX_HEIGHT intermediate nodes per item.
 *    it would look like the following if RADIX_TREE_MAX_HEIGHT=3.
 *
 *     root (t_height=3)
 *      |
 *      v
 *      [ | | | ]
 *           |
 *           v
 *           [ | | | ]
 *                |
 *                v
 *                [ | | | ]
 *                   |
 *                   v
 *                   p
 *
 * The height of tree (t_height) is dynamic.  It's smaller if only small
 * index values are used.  As an extreme case, if only index 0 is used,
 * the corresponding value is directly stored in the root of the tree
 * (struct radix_tree) without allocating any intermediate nodes.  In that
 * case, t_height=0.
 *
 * Gang lookup:
 *
 * This implementation provides a way to scan many nodes quickly via
 * radix_tree_gang_lookup_node function and its varients.
 *
 * Tags:
 *
 * This implementation provides tagging functionality, which allows quick
 * scanning of a subset of leaf nodes.  Leaf nodes are untagged when inserted
 * into the tree and can be tagged by radix_tree_set_tag function.
 * radix_tree_gang_lookup_tagged_node function and its variants returns only
 * leaf nodes with the given tag.  To reduce amount of nodes to visit for
 * these functions, this implementation keeps tagging information in internal
 * intermediate nodes and quickly skips uninterested parts of a tree.
 *
 * A tree has RADIX_TREE_TAG_ID_MAX independent tag spaces, each of which are
 * identified by an zero-origin numbers, tagid.  For the current implementation,
 * RADIX_TREE_TAG_ID_MAX is 2.  A set of tags is described as a bitmask tagmask,
 * which is a bitwise OR of (1 << tagid).
 */

/* Modified by Dylan Mayor for use in Unikraft */

//#include <sys/cdefs.h>

//#if defined(_KERNEL) || defined(_STANDALONE)
//__KERNEL_RCSID(0, "$NetBSD: radixtree.c,v 1.27 2020/05/14 08:34:19 msaitoh Exp $");
//#include <sys/param.h>
//#include <sys/errno.h>
//#include <sys/pool.h>
//#include <sys/radixtree.h>
//#include <lib/libkern/libkern.h>
//#if defined(_STANDALONE)
//#include <lib/libsa/stand.h>
//#endif /* defined(_STANDALONE) */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define KASSERT assert

#include <uk/radixtree.h>

#define	RADIX_TREE_BITS_PER_HEIGHT	4	/* XXX tune */
#define	RADIX_TREE_PTR_PER_NODE		(1 << RADIX_TREE_BITS_PER_HEIGHT)
#define	RADIX_TREE_MAX_HEIGHT		(64 / RADIX_TREE_BITS_PER_HEIGHT)
#define	RADIX_TREE_INVALID_HEIGHT	(RADIX_TREE_MAX_HEIGHT + 1)
#define	RADIX_TREE_TAG_MASK	((1 << RADIX_TREE_TAG_ID_MAX) - 1)

static inline void *
entry_ptr(void *p)
{

	return (void *)((uintptr_t)p & ~RADIX_TREE_TAG_MASK);
}

static inline unsigned int
entry_tagmask(void *p)
{

	return (uintptr_t)p & RADIX_TREE_TAG_MASK;
}

static inline void *
entry_compose(void *p, unsigned int tagmask)
{

	return (void *)((uintptr_t)p | tagmask);
}

static inline bool
entry_match_p(void *p, unsigned int tagmask)
{

	KASSERT(entry_ptr(p) != NULL || entry_tagmask(p) == 0);
	if (p == NULL) {
		return false;
	}
	if (tagmask == 0) {
		return true;
	}
	return (entry_tagmask(p) & tagmask) != 0;
}

/*
 * radix_tree_node: an intermediate node
 *
 * we don't care the type of leaf nodes.  they are just void *.
 *
 * we used to maintain a count of non-NULL nodes in this structure, but it
 * prevented it from being aligned to a cache line boundary; the performance
 * benefit from being cache friendly is greater than the benefit of having
 * a dedicated count value, especially in multi-processor situations where
 * we need to avoid intra-pool-page false sharing.
 */

struct radix_tree_node {
	void *n_ptrs[RADIX_TREE_PTR_PER_NODE];
};

/*
 * p_refs[0].pptr == &t->t_root
 *	:
 * p_refs[n].pptr == &(*p_refs[n-1])->n_ptrs[x]
 *	:
 *	:
 * p_refs[t->t_height].pptr == &leaf_pointer
 */

struct radix_tree_path {
	struct radix_tree_node_ref {
		void **pptr;
	} p_refs[RADIX_TREE_MAX_HEIGHT + 1]; /* +1 for the root ptr */
	/*
	 * p_lastidx is either the index of the last valid element of p_refs[]
	 * or RADIX_TREE_INVALID_HEIGHT.
	 * RADIX_TREE_INVALID_HEIGHT means that radix_tree_lookup_ptr found
	 * that the height of the tree is not enough to cover the given index.
	 */
	unsigned int p_lastidx;
};

static inline void **
path_pptr(const struct radix_tree *t, const struct radix_tree_path *p,
    unsigned int height)
{

	KASSERT(height <= t->t_height);
	return p->p_refs[height].pptr;
}

static inline struct radix_tree_node *
path_node(const struct radix_tree * t, const struct radix_tree_path *p,
    unsigned int height)
{

	KASSERT(height <= t->t_height);
	return entry_ptr(*path_pptr(t, p, height));
}

/*
 * radix_tree_init_tree:
 *
 * Initialize a tree.
 */

void
uk_radix_tree_init_tree(struct radix_tree *t)
{

	t->t_height = 0;
	t->t_root = NULL;
}

/*
 * radix_tree_fini_tree:
 *
 * Finish using a tree.
 */

void
uk_radix_tree_fini_tree(struct radix_tree *t)
{

	KASSERT(t->t_root == NULL);
	KASSERT(t->t_height == 0);
}

/*
 * radix_tree_empty_tree_p:
 *
 * Return if the tree is empty.
 */

bool
uk_radix_tree_empty_tree_p(struct radix_tree *t)
{

	return t->t_root == NULL;
}

/*
 * radix_tree_empty_tree_p:
 *
 * Return true if the tree has any nodes with the given tag.  Otherwise
 * return false.
 *
 * It's illegal to call this function with tagmask 0.
 */

bool
uk_radix_tree_empty_tagged_tree_p(struct radix_tree *t, unsigned int tagmask)
{

	KASSERT(tagmask != 0);
	return (entry_tagmask(t->t_root) & tagmask) == 0;
}

static void
radix_tree_node_init(struct radix_tree_node *n)
{

	memset(n, 0, sizeof(*n));
}

/*
 * radix_tree_sum_node:
 *
 * return the logical sum of all entries in the given node.  used to quickly
 * check for tag masks or empty nodes.
 */

static uintptr_t
radix_tree_sum_node(const struct radix_tree_node *n)
{
#if RADIX_TREE_PTR_PER_NODE > 16
	unsigned int i;
	uintptr_t sum;

	for (i = 0, sum = 0; i < RADIX_TREE_PTR_PER_NODE; i++) {
		sum |= (uintptr_t)n->n_ptrs[i];
	}
	return sum;
#else /* RADIX_TREE_PTR_PER_NODE > 16 */
	uintptr_t sum;

	/*
	 * Unrolling the above is much better than a tight loop with two
	 * test+branch pairs.  On x86 with gcc 5.5.0 this compiles into 19
	 * deterministic instructions including the "return" and prologue &
	 * epilogue.
	 */
	sum = (uintptr_t)n->n_ptrs[0];
	sum |= (uintptr_t)n->n_ptrs[1];
	sum |= (uintptr_t)n->n_ptrs[2];
	sum |= (uintptr_t)n->n_ptrs[3];
#if RADIX_TREE_PTR_PER_NODE > 4
	sum |= (uintptr_t)n->n_ptrs[4];
	sum |= (uintptr_t)n->n_ptrs[5];
	sum |= (uintptr_t)n->n_ptrs[6];
	sum |= (uintptr_t)n->n_ptrs[7];
#endif
#if RADIX_TREE_PTR_PER_NODE > 8
	sum |= (uintptr_t)n->n_ptrs[8];
	sum |= (uintptr_t)n->n_ptrs[9];
	sum |= (uintptr_t)n->n_ptrs[10];
	sum |= (uintptr_t)n->n_ptrs[11];
	sum |= (uintptr_t)n->n_ptrs[12];
	sum |= (uintptr_t)n->n_ptrs[13];
	sum |= (uintptr_t)n->n_ptrs[14];
	sum |= (uintptr_t)n->n_ptrs[15];
#endif
	return sum;
#endif /* RADIX_TREE_PTR_PER_NODE > 16 */
}

static int __unused
radix_tree_node_count_ptrs(const struct radix_tree_node *n)
{
	unsigned int i, c;

	for (i = c = 0; i < RADIX_TREE_PTR_PER_NODE; i++) {
		c += (n->n_ptrs[i] != NULL);
	}
	return c;
}

static struct radix_tree_node *
radix_tree_alloc_node(void)
{
	struct radix_tree_node *n;

#if defined(_KERNEL)
	/*
	 * note that pool_cache_get can block.
	 */
	n = pool_cache_get(radix_tree_node_cache, PR_NOWAIT);
#else /* defined(_KERNEL) */
#if defined(_STANDALONE)
	n = alloc(sizeof(*n));
#else /* defined(_STANDALONE) */
	n = malloc(sizeof(*n));
#endif /* defined(_STANDALONE) */
	if (n != NULL) {
		radix_tree_node_init(n);
	}
#endif /* defined(_KERNEL) */
	KASSERT(n == NULL || radix_tree_sum_node(n) == 0);
	return n;
}

static void
radix_tree_free_node(struct radix_tree_node *n)
{

	KASSERT(radix_tree_sum_node(n) == 0);
#if defined(_KERNEL)
	pool_cache_put(radix_tree_node_cache, n);
#elif defined(_STANDALONE)
	dealloc(n, sizeof(*n));
#else
	free(n);
#endif
}

/*
 * radix_tree_grow:
 *
 * increase the height of the tree.
 */

static /*__noinline*/ int
radix_tree_grow(struct radix_tree *t, unsigned int newheight)
{
	const unsigned int tagmask = entry_tagmask(t->t_root);
	struct radix_tree_node *newnodes[RADIX_TREE_MAX_HEIGHT];
	void *root;
	int h;

	KASSERT(newheight <= RADIX_TREE_MAX_HEIGHT);
	if ((root = t->t_root) == NULL) {
		t->t_height = newheight;
		return 0;
	}
	for (h = t->t_height; h < newheight; h++) {
		newnodes[h] = radix_tree_alloc_node();
		if (newnodes[h] == NULL) {
			while (--h >= (int)t->t_height) {
				newnodes[h]->n_ptrs[0] = NULL;
				radix_tree_free_node(newnodes[h]);
			}
			return ENOMEM;
		}
		newnodes[h]->n_ptrs[0] = root;
		root = entry_compose(newnodes[h], tagmask);
	}
	t->t_root = root;
	t->t_height = h;
	return 0;
}

/*
 * radix_tree_lookup_ptr:
 *
 * an internal helper function used for various exported functions.
 *
 * return the pointer to store the node for the given index.
 *
 * if alloc is true, try to allocate the storage.  (note for _KERNEL:
 * in that case, this function can block.)  if the allocation failed or
 * alloc is false, return NULL.
 *
 * if path is not NULL, fill it for the caller's investigation.
 *
 * if tagmask is not zero, search only for nodes with the tag set.
 * note that, however, this function doesn't check the tagmask for the leaf
 * pointer.  it's a caller's responsibility to investigate the value which
 * is pointed by the returned pointer if necessary.
 *
 * while this function is a bit large, as it's called with some constant
 * arguments, inlining might have benefits.  anyway, a compiler will decide.
 */

static inline void **

radix_tree_lookup_ptr(struct radix_tree *t, uint64_t idx,
    struct radix_tree_path *path, bool alloc, const unsigned int tagmask)
{
	struct radix_tree_node *n;
	int hshift = RADIX_TREE_BITS_PER_HEIGHT * t->t_height;
	int shift;
	void **vpp;
    uint64_t mask_val = 1;
    const uint64_t mask = (mask_val << RADIX_TREE_BITS_PER_HEIGHT) - 1;
	struct radix_tree_node_ref *refs = NULL;

	/*
	 * check unsupported combinations
	 */
	KASSERT(tagmask == 0 || !alloc);
	KASSERT(path == NULL || !alloc);
	vpp = &t->t_root;
	if (path != NULL) {
		refs = path->p_refs;
		refs->pptr = vpp;
	}
	n = NULL;
	for (shift = 64 - RADIX_TREE_BITS_PER_HEIGHT; shift >= 0;) {
		struct radix_tree_node *c;
		void *entry;
		const uint64_t i = (idx >> shift) & mask;

		if (shift >= hshift) {
			unsigned int newheight;

			KASSERT(vpp == &t->t_root);
			if (i == 0) {
				shift -= RADIX_TREE_BITS_PER_HEIGHT;
				continue;
			}
			if (!alloc) {
				if (path != NULL) {
					KASSERT((refs - path->p_refs) == 0);
					path->p_lastidx =
					    RADIX_TREE_INVALID_HEIGHT;
				}
				return NULL;
			}
			newheight = shift / RADIX_TREE_BITS_PER_HEIGHT + 1;
			if (radix_tree_grow(t, newheight)) {
				return NULL;
			}
			hshift = RADIX_TREE_BITS_PER_HEIGHT * t->t_height;
		}
		entry = *vpp;
		c = entry_ptr(entry);
		if (c == NULL ||
		    (tagmask != 0 &&
		    (entry_tagmask(entry) & tagmask) == 0)) {
			if (!alloc) {
				if (path != NULL) {
					path->p_lastidx = refs - path->p_refs;
				}
				return NULL;
			}
			c = radix_tree_alloc_node();
			if (c == NULL) {
				return NULL;
			}
			*vpp = c;
		}
		n = c;
		vpp = &n->n_ptrs[i];
		if (path != NULL) {
			refs++;
			refs->pptr = vpp;
		}
		shift -= RADIX_TREE_BITS_PER_HEIGHT;
	}
	if (alloc) {
		KASSERT(*vpp == NULL);
	}
	if (path != NULL) {
		path->p_lastidx = refs - path->p_refs;
	}
	return vpp;
}

/*
 * radix_tree_undo_insert_node:
 *
 * Undo the effects of a failed insert.  The conditions that led to the
 * insert may change and it may not be retried.  If the insert is not
 * retried, there will be no corresponding radix_tree_remove_node() for
 * this index in the future.  Therefore any adjustments made to the tree
 * before memory was exhausted must be reverted.
 */

static /*__noinline*/ void
radix_tree_undo_insert_node(struct radix_tree *t, uint64_t idx)
{
	struct radix_tree_path path;
	int i;

	(void)radix_tree_lookup_ptr(t, idx, &path, false, 0);
	if (path.p_lastidx == RADIX_TREE_INVALID_HEIGHT) {
		/*
		 * no nodes were inserted.
		 */
		return;
	}
	for (i = path.p_lastidx - 1; i >= 0; i--) {
		struct radix_tree_node ** const pptr =
		    (struct radix_tree_node **)path_pptr(t, &path, i);
		struct radix_tree_node *n;

		KASSERT(pptr != NULL);
		n = entry_ptr(*pptr);
		KASSERT(n != NULL);
		if (radix_tree_sum_node(n) != 0) {
			break;
		}
		radix_tree_free_node(n);
		*pptr = NULL;
	}
	/*
	 * fix up height
	 */
	if (i < 0) {
		KASSERT(t->t_root == NULL);
		t->t_height = 0;
	}
}

/*
 * radix_tree_insert_node:
 *
 * Insert the node at the given index.
 *
 * It's illegal to insert NULL.  It's illegal to insert a non-aligned pointer.
 *
 * This function returns ENOMEM if necessary memory allocation failed.
 * Otherwise, this function returns 0.
 *
 * Note that inserting a node can involves memory allocation for intermediate
 * nodes.  If _KERNEL, it's done with no-sleep IPL_NONE memory allocation.
 *
 * For the newly inserted node, all tags are cleared.
 */

int
uk_radix_tree_insert_node(struct radix_tree *t, uint64_t idx, void *p)
{
	void **vpp;

	KASSERT(p != NULL);
	KASSERT(entry_tagmask(entry_compose(p, 0)) == 0);
	vpp = radix_tree_lookup_ptr(t, idx, NULL, true, 0);
	if (vpp == NULL) {
		radix_tree_undo_insert_node(t, idx);
		return ENOMEM;
	}
	KASSERT(*vpp == NULL);
	*vpp = p;
	return 0;
}

/*
 * radix_tree_replace_node:
 *
 * Replace a node at the given index with the given node and return the
 * replaced one.
 *
 * It's illegal to try to replace a node which has not been inserted.
 *
 * This function keeps tags intact.
 */

void *
uk_radix_tree_replace_node(struct radix_tree *t, uint64_t idx, void *p)
{
	void **vpp;
	void *oldp;

	KASSERT(p != NULL);
	KASSERT(entry_tagmask(entry_compose(p, 0)) == 0);
	vpp = radix_tree_lookup_ptr(t, idx, NULL, false, 0);
	KASSERT(vpp != NULL);
	oldp = *vpp;
	KASSERT(oldp != NULL);
	*vpp = entry_compose(p, entry_tagmask(*vpp));
	return entry_ptr(oldp);
}

/*
 * radix_tree_remove_node:
 *
 * Remove the node at the given index.
 *
 * It's illegal to try to remove a node which has not been inserted.
 */

void *
uk_radix_tree_remove_node(struct radix_tree *t, uint64_t idx)
{
	struct radix_tree_path path;
	void **vpp;
	void *oldp;
	int i;

	vpp = radix_tree_lookup_ptr(t, idx, &path, false, 0);
	KASSERT(vpp != NULL);
	oldp = *vpp;
	KASSERT(oldp != NULL);
	KASSERT(path.p_lastidx == t->t_height);
	KASSERT(vpp == path_pptr(t, &path, path.p_lastidx));
	*vpp = NULL;
	for (i = t->t_height - 1; i >= 0; i--) {
		void *entry;
		struct radix_tree_node ** const pptr =
		    (struct radix_tree_node **)path_pptr(t, &path, i);
		struct radix_tree_node *n;

		KASSERT(pptr != NULL);
		entry = *pptr;
		n = entry_ptr(entry);
		KASSERT(n != NULL);
		if (radix_tree_sum_node(n) != 0) {
			break;
		}
		radix_tree_free_node(n);
		*pptr = NULL;
	}
	/*
	 * fix up height
	 */
	if (i < 0) {
		KASSERT(t->t_root == NULL);
		t->t_height = 0;
	}
	/*
	 * update tags
	 */
	for (; i >= 0; i--) {
		void *entry;
		struct radix_tree_node ** const pptr =
		    (struct radix_tree_node **)path_pptr(t, &path, i);
		struct radix_tree_node *n;
		unsigned int newmask;

		KASSERT(pptr != NULL);
		entry = *pptr;
		n = entry_ptr(entry);
		KASSERT(n != NULL);
		KASSERT(radix_tree_sum_node(n) != 0);
		newmask = radix_tree_sum_node(n) & RADIX_TREE_TAG_MASK;
		if (newmask == entry_tagmask(entry)) {
			break;
		}
		*pptr = entry_compose(n, newmask);
	}
	/*
	 * XXX is it worth to try to reduce height?
	 * if we do that, make radix_tree_grow rollback its change as well.
	 */
	return entry_ptr(oldp);
}

/*
 * radix_tree_lookup_node:
 *
 * Returns the node at the given index.
 * Returns NULL if nothing is found at the given index.
 */

void *
uk_radix_tree_lookup_node(struct radix_tree *t, uint64_t idx)
{
	void **vpp;

	vpp = radix_tree_lookup_ptr(t, idx, NULL, false, 0);
	if (vpp == NULL) {
		return NULL;
	}
	return entry_ptr(*vpp);
}

static inline void
gang_lookup_init(struct radix_tree *t, uint64_t idx,
    struct radix_tree_path *path, const unsigned int tagmask)
{
	void **vpp __unused;

	vpp = radix_tree_lookup_ptr(t, idx, path, false, tagmask);
	KASSERT(vpp == NULL ||
	    vpp == path_pptr(t, path, path->p_lastidx));
	KASSERT(&t->t_root == path_pptr(t, path, 0));
	KASSERT(path->p_lastidx == RADIX_TREE_INVALID_HEIGHT ||
	   path->p_lastidx == t->t_height ||
	   !entry_match_p(*path_pptr(t, path, path->p_lastidx), tagmask));
}

/*
 * gang_lookup_scan:
 *
 * a helper routine for radix_tree_gang_lookup_node and its variants.
 */

static inline unsigned int
__attribute__((__always_inline__))
gang_lookup_scan(struct radix_tree *t, struct radix_tree_path *path,
    void **results, const unsigned int maxresults, const unsigned int tagmask,
    const bool reverse, const bool dense)
{

	/*
	 * we keep the path updated only for lastidx-1.
	 * vpp is what path_pptr(t, path, lastidx) would be.
	 */
	void **vpp;
	unsigned int nfound;
	unsigned int lastidx;
	/*
	 * set up scan direction dependant constants so that we can iterate
	 * n_ptrs as the following.
	 *
	 *	for (i = first; i != guard; i += step)
	 *		visit n->n_ptrs[i];
	 */
	const int step = reverse ? -1 : 1;
	const unsigned int first = reverse ? RADIX_TREE_PTR_PER_NODE - 1 : 0;
	const unsigned int last = reverse ? 0 : RADIX_TREE_PTR_PER_NODE - 1;
	const unsigned int guard = last + step;

	KASSERT(maxresults > 0);
	KASSERT(&t->t_root == path_pptr(t, path, 0));
	lastidx = path->p_lastidx;
	KASSERT(lastidx == RADIX_TREE_INVALID_HEIGHT ||
	   lastidx == t->t_height ||
	   !entry_match_p(*path_pptr(t, path, lastidx), tagmask));
	nfound = 0;
	if (lastidx == RADIX_TREE_INVALID_HEIGHT) {
		/*
		 * requested idx is beyond the right-most node.
		 */
		if (reverse && !dense) {
			lastidx = 0;
			vpp = path_pptr(t, path, lastidx);
			goto descend;
		}
		return 0;
	}
	vpp = path_pptr(t, path, lastidx);
	while (/*CONSTCOND*/true) {
		struct radix_tree_node *n;
		unsigned int i;

		if (entry_match_p(*vpp, tagmask)) {
			KASSERT(lastidx == t->t_height);
			/*
			 * record the matching non-NULL leaf.
			 */
			results[nfound] = entry_ptr(*vpp);
			nfound++;
			if (nfound == maxresults) {
				return nfound;
			}
		} else if (dense) {
			return nfound;
		}
scan_siblings:
		/*
		 * try to find the next matching non-NULL sibling.
		 */
		if (lastidx == 0) {
			/*
			 * the root has no siblings.
			 * we've done.
			 */
			KASSERT(vpp == &t->t_root);
			break;
		}
		n = path_node(t, path, lastidx - 1);
		for (i = vpp - n->n_ptrs + step; i != guard; i += step) {
			KASSERT(i < RADIX_TREE_PTR_PER_NODE);
			if (entry_match_p(n->n_ptrs[i], tagmask)) {
				vpp = &n->n_ptrs[i];
				break;
			} else if (dense) {
				return nfound;
			}
		}
		if (i == guard) {
			/*
			 * not found.  go to parent.
			 */
			lastidx--;
			vpp = path_pptr(t, path, lastidx);
			goto scan_siblings;
		}
descend:
		/*
		 * following the left-most (or right-most in the case of
		 * reverse scan) child node, decend until reaching the leaf or
		 * an non-matching entry.
		 */
		while (entry_match_p(*vpp, tagmask) && lastidx < t->t_height) {
			/*
			 * save vpp in the path so that we can come back to this
			 * node after finishing visiting children.
			 */
			path->p_refs[lastidx].pptr = vpp;
			n = entry_ptr(*vpp);
			vpp = &n->n_ptrs[first];
			lastidx++;
		}
	}
	return nfound;
}

/*
 * radix_tree_gang_lookup_node:
 *
 * Scan the tree starting from the given index in the ascending order and
 * return found nodes.
 *
 * results should be an array large enough to hold maxresults pointers.
 * This function returns the number of nodes found, up to maxresults.
 * Returning less than maxresults means there are no more nodes in the tree.
 *
 * If dense == true, this function stops scanning when it founds a hole of
 * indexes.  I.e. an index for which radix_tree_lookup_node would returns NULL.
 * If dense == false, this function skips holes and continue scanning until
 * maxresults nodes are found or it reaches the limit of the index range.
 *
 * The result of this function is semantically equivalent to what could be
 * obtained by repeated calls of radix_tree_lookup_node with increasing index.
 * but this function is expected to be computationally cheaper when looking up
 * multiple nodes at once.  Especially, it's expected to be much cheaper when
 * node indexes are distributed sparsely.
 *
 * Note that this function doesn't return index values of found nodes.
 * Thus, in the case of dense == false, if index values are important for
 * a caller, it's the caller's responsibility to check them, typically
 * by examinining the returned nodes using some caller-specific knowledge
 * about them.
 * In the case of dense == true, a node returned via results[N] is always for
 * the index (idx + N).
 */

unsigned int
uk_radix_tree_gang_lookup_node(struct radix_tree *t, uint64_t idx,
    void **results, unsigned int maxresults, bool dense)
{
	struct radix_tree_path path;

	gang_lookup_init(t, idx, &path, 0);
	return gang_lookup_scan(t, &path, results, maxresults, 0, false, dense);
}

/*
 * radix_tree_gang_lookup_node_reverse:
 *
 * Same as radix_tree_gang_lookup_node except that this one scans the
 * tree in the reverse order.  I.e. descending index values.
 */

unsigned int
uk_radix_tree_gang_lookup_node_reverse(struct radix_tree *t, uint64_t idx,
    void **results, unsigned int maxresults, bool dense)
{
	struct radix_tree_path path;

	gang_lookup_init(t, idx, &path, 0);
	return gang_lookup_scan(t, &path, results, maxresults, 0, true, dense);
}

/*
 * radix_tree_gang_lookup_tagged_node:
 *
 * Same as radix_tree_gang_lookup_node except that this one only returns
 * nodes tagged with tagid.
 *
 * It's illegal to call this function with tagmask 0.
 */

unsigned int
uk_radix_tree_gang_lookup_tagged_node(struct radix_tree *t, uint64_t idx,
    void **results, unsigned int maxresults, bool dense, unsigned int tagmask)
{
	struct radix_tree_path path;

	KASSERT(tagmask != 0);
	gang_lookup_init(t, idx, &path, tagmask);
	return gang_lookup_scan(t, &path, results, maxresults, tagmask, false,
	    dense);
}

/*
 * radix_tree_gang_lookup_tagged_node_reverse:
 *
 * Same as radix_tree_gang_lookup_tagged_node except that this one scans the
 * tree in the reverse order.  I.e. descending index values.
 */

unsigned int
uk_radix_tree_gang_lookup_tagged_node_reverse(struct radix_tree *t, uint64_t idx,
    void **results, unsigned int maxresults, bool dense, unsigned int tagmask)
{
	struct radix_tree_path path;

	KASSERT(tagmask != 0);
	gang_lookup_init(t, idx, &path, tagmask);
	return gang_lookup_scan(t, &path, results, maxresults, tagmask, true,
	    dense);
}

/*
 * radix_tree_get_tag:
 *
 * Return the tagmask for the node at the given index.
 *
 * It's illegal to call this function for a node which has not been inserted.
 */

unsigned int
uk_radix_tree_get_tag(struct radix_tree *t, uint64_t idx, unsigned int tagmask)
{
	/*
	 * the following two implementations should behave same.
	 * the former one was chosen because it seems faster.
	 */
#if 1
	void **vpp;

	vpp = radix_tree_lookup_ptr(t, idx, NULL, false, tagmask);
	if (vpp == NULL) {
		return false;
	}
	KASSERT(*vpp != NULL);
	return (entry_tagmask(*vpp) & tagmask);
#else
	void **vpp;

	vpp = radix_tree_lookup_ptr(t, idx, NULL, false, 0);
	KASSERT(vpp != NULL);
	return (entry_tagmask(*vpp) & tagmask);
#endif
}

/*
 * radix_tree_set_tag:
 *
 * Set the tag for the node at the given index.
 *
 * It's illegal to call this function for a node which has not been inserted.
 * It's illegal to call this function with tagmask 0.
 */

void
uk_radix_tree_set_tag(struct radix_tree *t, uint64_t idx, unsigned int tagmask)
{
	struct radix_tree_path path;
	void **vpp __unused;
	int i;

	KASSERT(tagmask != 0);
	vpp = radix_tree_lookup_ptr(t, idx, &path, false, 0);
	KASSERT(vpp != NULL);
	KASSERT(*vpp != NULL);
	KASSERT(path.p_lastidx == t->t_height);
	KASSERT(vpp == path_pptr(t, &path, path.p_lastidx));
	for (i = t->t_height; i >= 0; i--) {
		void ** const pptr = (void **)path_pptr(t, &path, i);
		void *entry;

		KASSERT(pptr != NULL);
		entry = *pptr;
		if ((entry_tagmask(entry) & tagmask) != 0) {
			break;
		}
		*pptr = (void *)((uintptr_t)entry | tagmask);
	}
}

/*
 * radix_tree_clear_tag:
 *
 * Clear the tag for the node at the given index.
 *
 * It's illegal to call this function for a node which has not been inserted.
 * It's illegal to call this function with tagmask 0.
 */

void
uk_radix_tree_clear_tag(struct radix_tree *t, uint64_t idx, unsigned int tagmask)
{
	struct radix_tree_path path;
	void **vpp;
	int i;

	KASSERT(tagmask != 0);
	vpp = radix_tree_lookup_ptr(t, idx, &path, false, 0);
	KASSERT(vpp != NULL);
	KASSERT(*vpp != NULL);
	KASSERT(path.p_lastidx == t->t_height);
	KASSERT(vpp == path_pptr(t, &path, path.p_lastidx));
	/*
	 * if already cleared, nothing to do
	 */
	if ((entry_tagmask(*vpp) & tagmask) == 0) {
		return;
	}
	/*
	 * clear the tag only if no children have the tag.
	 */
	for (i = t->t_height; i >= 0; i--) {
		void ** const pptr = (void **)path_pptr(t, &path, i);
		void *entry;

		KASSERT(pptr != NULL);
		entry = *pptr;
		KASSERT((entry_tagmask(entry) & tagmask) != 0);
		*pptr = entry_compose(entry_ptr(entry),
		    entry_tagmask(entry) & ~tagmask);
		/*
		 * check if we should proceed to process the next level.
		 */
		if (0 < i) {
			struct radix_tree_node *n = path_node(t, &path, i - 1);

			if ((radix_tree_sum_node(n) & tagmask) != 0) {
				break;
			}
		}
	}
}
