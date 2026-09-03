#include <stdint.h>
#include <stddef.h>

const uint8_t core1_blob[] = {
#include <core1_blob.bin.inc>
};

const size_t core1_blob_size = sizeof(core1_blob);
