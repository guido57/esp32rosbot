#ifndef PSRAM_ALLOCATOR_H
#define PSRAM_ALLOCATOR_H

#include <memory>
#include <esp_heap_caps.h>


// Custom allocator for PSRAM
template <typename T>
struct PsramAllocator {
    using value_type = T;

    PsramAllocator() = default;

    template <typename U>
    constexpr PsramAllocator(const PsramAllocator<U>&) noexcept {}

    [[nodiscard]] T* allocate(std::size_t n) {
        T* ptr = static_cast<T*>(heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM));
        if (!ptr) {
            printf("Failed to allocate %d bytes in PSRAM!\r\n", n * sizeof(T));
            throw std::bad_alloc();
        }
        // printf("Allocated %d bytes in PSRAM at address %p\r\n", n * sizeof(T), ptr);
        return ptr;
    }

    void deallocate(T* p, std::size_t) noexcept {
        heap_caps_free(p);
    }
};

template <typename T, typename U>
bool operator==(const PsramAllocator<T>&, const PsramAllocator<U>&) { return true; }

template <typename T, typename U>
bool operator!=(const PsramAllocator<T>&, const PsramAllocator<U>&) { return false; }

#endif // PSRAM_ALLOCATOR_H