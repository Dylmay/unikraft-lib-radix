#ifndef UKRADIX_RADIXTREE_H
#define UKRADIX_RADIXTREE_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* Modified by Dylan Mayor for use in Unikraft */

#include <stdbool.h>
#include <stdint.h>

struct radix_tree {
    void *t_root;
    unsigned int t_height;
};

/*
* tree
*/
void uk_radix_tree_init_tree(struct radix_tree *);
void uk_radix_tree_fini_tree(struct radix_tree *);
bool uk_radix_tree_empty_tree_p(struct radix_tree *);

/*
* node
*/
int uk_radix_tree_insert_node(struct radix_tree *, uint64_t, void *);
void *uk_radix_tree_replace_node(struct radix_tree *, uint64_t, void *);
void *uk_radix_tree_remove_node(struct radix_tree *, uint64_t);
void *uk_radix_tree_lookup_node(struct radix_tree *, uint64_t);
unsigned int uk_radix_tree_gang_lookup_node(struct radix_tree *, uint64_t,
                                         void **, unsigned int, bool);
unsigned int uk_radix_tree_gang_lookup_node_reverse(struct radix_tree *, uint64_t,
                                                 void **, unsigned int, bool);
/*
* tag
*/
typedef unsigned int radix_tree_tagmask_t;
#define	RADIX_TREE_TAG_ID_MAX	2
radix_tree_tagmask_t uk_radix_tree_get_tag(struct radix_tree *, uint64_t,
                                        radix_tree_tagmask_t);
void uk_radix_tree_set_tag(struct radix_tree *, uint64_t, radix_tree_tagmask_t);
void uk_radix_tree_clear_tag(struct radix_tree *, uint64_t, radix_tree_tagmask_t);
unsigned int uk_radix_tree_gang_lookup_tagged_node(struct radix_tree *, uint64_t,
                                                void **, unsigned int, bool, radix_tree_tagmask_t);
unsigned int uk_radix_tree_gang_lookup_tagged_node_reverse(struct radix_tree *,
                                                        uint64_t, void **, unsigned int, bool, radix_tree_tagmask_t);
bool uk_radix_tree_empty_tagged_tree_p(struct radix_tree *, radix_tree_tagmask_t);

#ifdef __cplusplus
}
#endif
#endif //UKRADIX_RADIXTREE_H
