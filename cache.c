#include "cache.h"
#include "dogfault.h"
#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// DO NOT MODIFY THIS FILE. INVOKE AFTER EACH ACCESS FROM runTrace
void print_result(result r) {
  if (r.status == CACHE_EVICT)
    printf(" [status: miss eviction, victim_block: 0x%llx, insert_block: 0x%llx]",
           r.victim_block_addr, r.insert_block_addr);
  if (r.status == CACHE_HIT)
    printf(" [status: hit]");
  if (r.status == CACHE_MISS)
    printf(" [status: miss, insert_block: 0x%llx]", r.insert_block_addr);
}

/* This is the entry point to operate the cache for a given address in the trace file.
 * First, is increments the global lru_clock in the corresponding cache set for the address.
 * Second, it checks if the address is already in the cache using the "probe_cache" function.
 * If yes, it is a cache hit:
 *     1) call the "hit_cacheline" function to update the counters inside the hit cache 
 *        line, including its lru_clock and access_counter.
 *     2) record a hit status in the return "result" struct and update hit_count 
 * Otherwise, it is a cache miss:
 *     1) call the "insert_cacheline" function, trying to find an empty cache line in the
 *        cache set and insert the address into the empty line. 
 *     2) if the "insert_cacheline" function returns true, record a miss status and the
          inserted block address in the return "result" struct and update miss_count
 *     3) otherwise, if the "insert_cacheline" function returns false:
 *          a) call the "victim_cacheline" function to figure which victim cache line to 
 *             replace based on the cache replacement policy (LRU and LFU).
 *          b) call the "replace_cacheline" function to replace the victim cache line with
 *             the new cache line to insert.
 *          c) record an eviction status, the victim block address, and the inserted block
 *             address in the return "result" struct. Update miss_count and eviction_count.
 */
result operateCache(const unsigned long long address, Cache *cache) {
  /* YOUR CODE HERE */
    result r;

    // 1) Find the set index and increment that set's global lru_clock
    unsigned long long set_idx = cache_set(address, cache);
    Set *set = &cache->sets[set_idx];
    set->lru_clock++;  // increment global clock for this set

    // 2) Check if address is already in cache (hit)
    if (probe_cache(address, cache)) {
        hit_cacheline(address, cache);
        r.status = CACHE_HIT;
        r.victim_block_addr = 0;  // no eviction
        r.insert_block_addr = address_to_block(address, cache);
        cache->hit_count++;
    } else {
        // 3) Miss: try to insert into an empty line first
        bool inserted = insert_cacheline(address, cache);

        if (inserted) {
            r.status = CACHE_MISS;
            r.victim_block_addr = 0;
            r.insert_block_addr = address_to_block(address, cache);
            cache->miss_count++;
        } else {
            // 4) No empty line, need to evict one victim line
            unsigned long long victim_blk = victim_cacheline(address, cache);

            replace_cacheline(victim_blk, address, cache);

            r.status = CACHE_EVICT;
            r.victim_block_addr = victim_blk;
            r.insert_block_addr = address_to_block(address, cache);
            cache->miss_count++;
            cache->eviction_count++;
        }
    }

    return r;
}

// HELPER FUNCTIONS USEFUL FOR IMPLEMENTING THE CACHE
// Given an address, return the block (aligned) address,
// i.e., byte offset bits are cleared to 0
unsigned long long address_to_block(const unsigned long long address,
                                const Cache *cache) {
  /* YOUR CODE HERE */
  return address & ~((1ULL << cache->blockBits) - 1ULL);
}

// Return the cache tag of an address
unsigned long long cache_tag(const unsigned long long address,
                             const Cache *cache) {
  /* YOUR CODE HERE */
  return address >> (cache->setBits + cache->blockBits);}

// Return the cache set index of the address
unsigned long long cache_set(const unsigned long long address,
                             const Cache *cache) {
  /* YOUR CODE HERE */
  return (address >> cache->blockBits) & ((1ULL << cache->setBits) - 1ULL);
}

// Check if the address is found in the cache. If so, return true. else return false.
bool probe_cache(const unsigned long long address, const Cache *cache) {
  /* YOUR CODE HERE */
  unsigned long long set_idx = cache_set(address, cache);
    unsigned long long tag     = cache_tag(address, cache);

    const Set *set = &cache->sets[set_idx];

    for (int i = 0; i < cache->linesPerSet; ++i) {
        const Line *line = &set->lines[i];
        if (line->valid && line->tag == tag)
            return true;              // hit found
    }
  return false;
}

// Access address in cache. Called only if probe is successful.
// Update the LRU (least recently used) or LFU (least frequently used) counters.
void hit_cacheline(const unsigned long long address, Cache *cache){
  /* YOUR CODE HERE */
  unsigned long long set_idx = cache_set(address, cache);
    unsigned long long tag     = cache_tag(address, cache);

    Set *set = &cache->sets[set_idx];

    for (int i = 0; i < cache->linesPerSet; ++i) {
        Line *line = &set->lines[i];
        if (line->valid && line->tag == tag) {
            line->lru_clock = set->lru_clock;  // newest access time
            ++line->access_counter;            // bump LFU count
            break;
        }
    }
}

/* This function is only called if probe_cache returns false, i.e., the address is
 * not in the cache. In this function, it will try to find an empty (i.e., invalid)
 * cache line for the address to insert. 
 * If it found an empty one:
 *     1) it inserts the address into that cache line (marking it valid).
 *     2) it updates the cache line's lru_clock based on the global lru_clock 
 *        in the cache set and initiates the cache line's access_counter.
 *     3) it returns true.
 * Otherwise, it returns false.  
 */ 
bool insert_cacheline(const unsigned long long address, Cache *cache) {
  /* YOUR CODE HERE */
    unsigned long long set_idx = cache_set(address, cache);
    unsigned long long tag     = cache_tag(address, cache);
    unsigned long long blk_addr = address_to_block(address, cache);

    Set *set = &cache->sets[set_idx];

    /* Scan for an invalid (empty) line */
    for (int i = 0; i < cache->linesPerSet; ++i) {
        Line *line = &set->lines[i];
        if (!line->valid) {
            /* Found a free slot – fill it in */
            line->valid          = true;
            line->tag            = tag;
            line->block_addr     = blk_addr;
            line->lru_clock      = set->lru_clock;  // most‑recent access time
            line->access_counter = 1;               // first access

            return true;  // successfully inserted
        }
    }

    /* No empty line in this set */
    return false;
}

// If there is no empty cacheline, this method figures out which cacheline to replace
// depending on the cache replacement policy (LRU and LFU). It returns the block address
// of the victim cacheline; note we no longer have access to the full address of the victim
unsigned long long victim_cacheline(const unsigned long long address,
                                const Cache *cache) {
  /* YOUR CODE HERE */
    unsigned long long set_idx = cache_set(address, cache);
    const Set *set = &cache->sets[set_idx];

    int victim = 0;  // index of candidate victim line

    if (cache->lfu) {
        /* ---------------- LFU replacement ---------------- */
        int              fewest  = set->lines[0].access_counter;
        unsigned long long oldest = set->lines[0].lru_clock;  // tie‑breaker

        for (int i = 1; i < cache->linesPerSet; ++i) {
            const Line *l = &set->lines[i];
            if (l->access_counter < fewest ||
               (l->access_counter == fewest && l->lru_clock < oldest)) {
                victim  = i;
                fewest  = l->access_counter;
                oldest  = l->lru_clock;
            }
        }
    } else {
        /* ---------------- LRU replacement ---------------- */
        unsigned long long oldest = set->lines[0].lru_clock;

        for (int i = 1; i < cache->linesPerSet; ++i) {
            if (set->lines[i].lru_clock < oldest) {
                oldest = set->lines[i].lru_clock;
                victim = i;
            }
        }
    }

    return set->lines[victim].block_addr;  // block address of the chosen victim
}

/* Replace the victim cacheline with the new address to insert. Note for the victim cachline,
 * we only have its block address. For the new address to be inserted, we have its full address.
 * Remember to update the new cache line's lru_clock based on the global lru_clock in the cache
 * set and initiate the cache line's access_counter.
 */
void replace_cacheline(const unsigned long long victim_block_addr,
		       const unsigned long long insert_addr, Cache *cache) {
  /* YOUR CODE HERE */
    unsigned long long set_idx   = cache_set(insert_addr, cache);
    unsigned long long insert_tag = cache_tag(insert_addr, cache);
    unsigned long long insert_block = address_to_block(insert_addr, cache);

    Set *set = &cache->sets[set_idx];

    // Find the victim line by block address
    for (int i = 0; i < cache->linesPerSet; ++i) {
        Line *line = &set->lines[i];

        if (line->valid && line->block_addr == victim_block_addr) {
            // Replace this line with the new one
            line->tag            = insert_tag;
            line->block_addr     = insert_block;
            line->lru_clock      = set->lru_clock;
            line->access_counter = 1;      // reset counter for LFU
            return;
        }
    }

    // Should never get here
    fprintf(stderr, "Error: victim block not found in replace_cacheline\n");
    exit(1);
}

// allocate the memory space for the cache with the given cache parameters
// and initialize the cache sets and lines.
// Initialize the cache name to the given name 
void cacheSetUp(Cache *cache, char *name) {
  /* YOUR CODE HERE */
    int numSets = 1 << cache->setBits;

    // Allocate memory for the sets
    cache->sets = (Set *)calloc(numSets, sizeof(Set));
    assert(cache->sets != NULL);  // fail fast if memory runs out

    for (int i = 0; i < numSets; ++i) {
        // Allocate space for each set's lines
        cache->sets[i].lines = (Line *)calloc(cache->linesPerSet, sizeof(Line));
        assert(cache->sets[i].lines != NULL);

        // Initialize LRU clock of the set
        cache->sets[i].lru_clock = 0;

        // Lines are already zeroed by calloc (valid = false, etc.)
    }

    // Initialize counters and name
    cache->hit_count      = 0;
    cache->miss_count     = 0;
    cache->eviction_count = 0;

    cache->name = name;  // store the pointer (assume name stays valid)
}

// deallocate the memory space for the cache
void deallocate(Cache *cache) {
  /* YOUR CODE HERE */
    if (!cache || !cache->sets)
        return;  // nothing to free

    int numSets = 1 << cache->setBits;

    for (int i = 0; i < numSets; ++i) {
        free(cache->sets[i].lines);  // free each set's lines array
        cache->sets[i].lines = NULL;
    }

    free(cache->sets);  // free the array of sets
    cache->sets = NULL;

    // Optional: reset counters and other fields
    cache->hit_count = 0;
    cache->miss_count = 0;
    cache->eviction_count = 0;
}

// print out summary stats for the cache
void printSummary(const Cache *cache) {
  printf("%s hits: %d, misses: %d, evictions: %d\n", cache->name, cache->hit_count,
         cache->miss_count, cache->eviction_count);
}
