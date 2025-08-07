#include "dogfault.h"
#include "cache.h"
#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "config.h"

// Helper to compute base^exponent (integer power)
unsigned long long power(int base, int exponent) {
    // Initialize result to 1 (any number to the power of 0 is 1)
    unsigned long long result = 1;
    // Loop until exponent is reduced to 0
    while (exponent > 0) {
        // Multiply result by base in each iteration
        result *= base;
        // Decrease the exponent by 1
        exponent--;
    }
    // Return the final computed power value
    return result;
}

// Extract block address from full address by masking off block offset bits
unsigned long long address_to_block(const unsigned long long addr, const Cache *cache) {
    return (addr >> cache->blockBits) << cache->blockBits;
}

// Get tag bits from address
unsigned long long cache_tag(const unsigned long long addr, const Cache *cache) {
    return addr >> (cache->blockBits + cache->setBits);
}

// Get set index bits from address
unsigned long long cache_set(const unsigned long long addr, const Cache *cache) {
    unsigned int mask = (1 << cache->setBits) - 1;
    return (addr >> cache->blockBits) & mask;
}

// Checks if cache contains a line matching the address (valid + tag match)
bool probe_cache(const unsigned long long addr, const Cache *cache) {
    // Extract the tag bits from the address using cache's configuration
    unsigned long long tagVal = cache_tag(addr, cache);
    // Extract the set index bits from the address using cache's configuration
    unsigned long long setIdx = cache_set(addr, cache);
    Set *set = &cache->sets[setIdx]; // Get a pointer to the set corresponding to the calculated set index

    // Loop through each line in the selected set
    for (int i = 0; i < cache->linesPerSet; i++) {
        // Get a pointer to the current line
        Line *line = &set->lines[i];

        // Check if the line is valid AND the tag matches the extracted tag
        if (line->valid && line->tag == tagVal) {
            // It is a cache hit, so return true
            return true;
        }
    }
    return false; // No valid line with matching tag found in the set — cache miss
}

// Updates metadata on a cache hit for either LFU or LRU policies
void hit_cacheline(const unsigned long long addr, Cache *cache) {
    // Extract the tag bits from the given address
    unsigned long long tagVal = cache_tag(addr, cache);
    
    // Extract the set index from the given address
    unsigned long long setIdx = cache_set(addr, cache);
    
    // Get a pointer to the set corresponding to the calculated index
    Set *set = &cache->sets[setIdx];

    // Iterate through all lines in the set to find the matching one
    for (int i = 0; i < cache->linesPerSet; i++) {
        // Get a pointer to the current line
        Line *line = &set->lines[i];

        // Check if the line is valid and the tag matches
        if (line->valid && line->tag == tagVal) {
            // If LFU policy is enabled, increment the access counter
            if (cache->lfu) {
                line->access_counter++;
            } else {
                // Otherwise, update the line's LRU clock to the current set's clock
                line->lru_clock = set->lru_clock;
            }

            // Increment the set's LRU clock to reflect a new access
            set->lru_clock++;

             // Exit the loop early since the matching line has been found and updated
            break;
        }
    }
}

// Attempts to insert a new cache line if an invalid slot is available
bool insert_cacheline(const unsigned long long addr, Cache *cache) {
    // Extract the tag from the address based on cache configuration
    unsigned long long tagVal = cache_tag(addr, cache);
    
    // Extract the set index from the address
    unsigned long long setIdx = cache_set(addr, cache);
    Set *set = &cache->sets[setIdx]; // Get a pointer to the set where the address maps

    // Loop through all lines in the set to find an invalid (empty) slot
    for (int i = 0; i < cache->linesPerSet; i++) {
        // Get a pointer to the current line
        Line *line = &set->lines[i];
        
        // If the line is invalid, we can insert the new block here
        if (!line->valid) {
            line->valid = true; // Mark the line as valid since it's now occupied
            line->tag = tagVal; // Store the extracted tag for comparison during future accesses
            line->block_addr = address_to_block(addr, cache); // Store the block address (aligned address without offset bits)
            line->lru_clock = set->lru_clock; // Record the current LRU clock value for LRU tracking
            line->access_counter = 1; // Set access counter to 1 for LFU tracking
            set->lru_clock++; // Advance the set’s LRU clock (indicates time/progress)
            return true; // Insertion successful
        }
    }
    return false; // No invalid line found — insertion failed (must evict)
}

// Determines the victim cache line to evict in the set that corresponds to the given address
unsigned long long victim_cacheline(const unsigned long long addr, const Cache *cache) {
    // Get the set index from the address
    unsigned long long setIdx = cache_set(addr, cache);

    // Get a pointer to the corresponding set
    Set *set = &cache->sets[setIdx];

    // Initialize the minimum access counter to max possible value (for LFU)
    unsigned long long minAccessCount = (unsigned long long)(-1);

    // Initialize the minimum LRU clock value to max possible value (for LRU tie-breakers)
    unsigned long long minLruClock = (unsigned long long)(-1);

    // Pointer to track the chosen victim line
    Line *victim = NULL;

    // Iterate over all lines in the set to find the one to evict
    for (int i = 0; i < cache->linesPerSet; i++) {
        // Get a pointer to the current line
        Line *line = &set->lines[i];

        // If LFU policy is enabled
        if (cache->lfu) {
            // Prefer the line with the smallest access count
            if (line->access_counter < minAccessCount) {
                minAccessCount = line->access_counter;
                minLruClock = line->lru_clock;
                victim = line;
            }
            // If access counts are equal, use LRU clock as a tiebreaker
            else if (line->access_counter == minAccessCount) {
                if (line->lru_clock < minLruClock) {
                    minLruClock = line->lru_clock;
                    victim = line;
                }
            }
        }
        // If LRU policy is used
        else {
            // Select the line with the oldest (smallest) LRU clock value
            if (line->lru_clock < minLruClock) {
                minLruClock = line->lru_clock;
                victim = line;
            }
        }
    }

    // Ensure a victim line was found (safety check)
    assert(victim != NULL);

    // Return the block address of the chosen victim line
    return victim->block_addr;
}

// Replace victim cache line with new block from insert_addr
void replace_cacheline(const unsigned long long victim_block_addr, const unsigned long long insert_addr, Cache *cache) {
    // Determine the index of the set in which replacement will occur
    unsigned long long setIdx = cache_set(insert_addr, cache);
    Set *set = &cache->sets[setIdx];

    Line *victim = NULL;

    // Search for the victim line in the set by matching the block address
    for (int i = 0; i < cache->linesPerSet; i++) {
        Line *line = &set->lines[i];
        if (line->block_addr == victim_block_addr) {
            victim = line;
            break;
        }
    }

    // Ensure a valid victim was found
    assert(victim != NULL);

    // Replace victim line's contents with new block's metadata
    victim->block_addr = address_to_block(insert_addr, cache); // update block address
    victim->tag = cache_tag(insert_addr, cache);               // update tag
    victim->valid = true;                                      // mark line as valid
    victim->lru_clock = set->lru_clock;                        // update LRU clock
    victim->access_counter = 1;                                // reset access counter (used for LFU)
    set->lru_clock++;                                          // increment set-wide LRU clock
}

// Initialize cache data structure and allocate sets/lines
void cacheSetUp(Cache *cache, char *name) {
    // Set the number of lines per set in the cache (from macro)
    cache->linesPerSet = CACHE_LINES_PER_SET;

    // Set the number of block offset bits (defines block size)
    cache->blockBits = CACHE_BLOCK_BITS;

    // Set the number of set index bits (defines number of sets)
    cache->setBits = CACHE_SET_BITS;

    // Set the cache replacement policy to LFU (Least Frequently Used)
    cache->lfu = CACHE_LFU;

    // Calculate the number of sets: 2^setBits
    unsigned long long numSets = power(2, cache->setBits);

    // Store number of lines per set
    unsigned long long numLines = cache->linesPerSet;

    // Allocate memory for all sets in the cache
    cache->sets = (Set *)malloc(sizeof(Set) * numSets);

    // Loop through each set
    for (unsigned long long i = 0; i < numSets; i++) {
        // Initialize LRU clock for the set
        cache->sets[i].lru_clock = 0;

        // Allocate memory for all lines in the current set
        cache->sets[i].lines = (Line *)malloc(sizeof(Line) * numLines);

        // Loop through each line in the set
        for (unsigned long long j = 0; j < numLines; j++) {
            // Mark line as invalid (empty)
            cache->sets[i].lines[j].valid = false;

            // Initialize LRU clock for the line
            cache->sets[i].lines[j].lru_clock = 0;

            // Initialize access counter for LFU replacement
            cache->sets[i].lines[j].access_counter = 0;
        }
    }

    // Initialize cache statistics
    cache->hit_count = 0;
    cache->miss_count = 0;
    cache->eviction_count = 0;

    // Force LFU enabled regardless of default (can override later)
    cache->lfu = 1;
}

// Free the dynamically allocated cache memory
void deallocate(Cache *cache) {
    // Calculate the total number of sets in the cache using 2^setBits
    unsigned long long numSets = power(2, cache->setBits);

    // Loop through each set and free the array of cache lines
    for (unsigned long long i = 0; i < numSets; i++) {
        free(cache->sets[i].lines); // Free memory allocated for lines in set i
    }

    // Finally, free the array of sets
    free(cache->sets);
}


// Main cache operation: probe, insert, or evict+replace accordingly
result operateCache(const unsigned long long addr, Cache *cache) {
    result r = {0};  // Initialize result structure to store operation status

    // Calculate the index of the cache set for this address
    unsigned long long setIdx = cache_set(addr, cache);

    // Increment the LRU clock for the set to help with replacement policy
    cache->sets[setIdx].lru_clock++;

    // CASE 1: Cache hit
    if (probe_cache(addr, cache)) {
        #ifdef PRINT_CACHE_TRACES
        printf(CACHE_HIT_FORMAT, addr);  // Print hit trace if enabled
        #endif
        hit_cacheline(addr, cache);      // Update metadata for hit (e.g., LRU)
        r.status = CACHE_HIT;            // Set result status
        cache->hit_count++;              // Increment hit counter

    // CASE 2: Cache miss with space to insert
    } else if (insert_cacheline(addr, cache)) {
        #ifdef PRINT_CACHE_TRACES
        printf(CACHE_MISS_FORMAT, addr); // Print miss trace if enabled
        #endif
        r.status = CACHE_MISS;           // Set result status
        r.insert_block_addr = address_to_block(addr, cache);  // Store block address inserted
        cache->miss_count++;             // Increment miss counter

    // CASE 3: Cache miss and set is full, requires eviction
    } else {
        #ifdef PRINT_CACHE_TRACES
        printf(CACHE_EVICTION_FORMAT, addr); // Print eviction trace if enabled
        #endif
        r.victim_block_addr = victim_cacheline(addr, cache); // Get address of block being evicted
        r.insert_block_addr = address_to_block(addr, cache); // Address of block to insert
        replace_cacheline(r.victim_block_addr, r.insert_block_addr, cache); // Replace the block
        r.status = CACHE_EVICT;          // Set result status
        cache->eviction_count++;         // Increment eviction counter
        cache->miss_count++;             // Still a miss
    }

    return r;  // Return the result of the cache operation
}

// Process a cache operation and return the latency based on the cache result
int processCacheOperation(unsigned long addr, Cache *cache) {
    // Perform the cache operation: probe, insert, or replace
    result r = operateCache(addr, cache);

    // If the operation was a cache hit, return the hit latency
    if (r.status == CACHE_HIT) {
        return CACHE_HIT_LATENCY;

    // If the operation was a cache miss (but no eviction), return the miss latency
    } else if (r.status == CACHE_MISS) {
        return CACHE_MISS_LATENCY;

    // For any other status (usually eviction), return the corresponding latency
    } else {
        return CACHE_OTHER_LATENCY;
    }
}
