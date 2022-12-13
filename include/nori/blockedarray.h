#if !defined(__NORI_BLOCKEDARRAY_H)
#define __NORI_BLOCKEDARRAY_H

#include <nori/common.h>

NORI_NAMESPACE_BEGIN

template <typename T, int logBlockSize> 
class BlockedArray {
public:
    BlockedArray(int uRes, int vRes, const T *d = nullptr)
            : m_uRes(uRes), m_vRes(vRes), m_uBlocks(RoundUp(uRes) >> logBlockSize) {

        // Prepare data allocation
        int nAlloc = RoundUp(m_uRes) * RoundUp(m_vRes);
        m_data = AllocAligned<T>(nAlloc);

        // Allocate space for each block of data in the array
        for (int i = 0; i < nAlloc; ++i) {
            new (&m_data[i]) T();
        }

        // Check for given data and copy into this if needed
        if (d) {
            for (int v = 0; v < m_vRes; ++v) {
                for (int u = 0; u < m_uRes; ++u) {
                    (*this)(u, v) = d[v * m_uRes + u];
                }
            }
        }
    }

    // Getter for the preset blocksize
    constexpr int BlockSize() const { 
        return 1 << logBlockSize; 
    }

    // Utility method that rounds up to the next block
    int RoundUp(int x) const { 
        return (x + BlockSize() - 1) & ~(BlockSize() - 1); 
    }

    // ==== Getters for u and v resolutions ====
    int uSize() const { 
        return m_uRes; 
    }

    int vSize() const { 
        return m_vRes; 
    }
    //==========================================

    // Destructor for the blocked array
    ~BlockedArray() {
        for (int i = 0; i < m_uRes * m_vRes; ++i) {
            data[i].~T();
        }
        FreeAligned(data);
    }

    // Getter for a specific block in the given int 
    int Block(int a) const {
        return a >> logBlockSize; 
    }

    // Offset of the given int
    int Offset(int a) const { 
        return (a & (BlockSize() - 1)); 
    }

    // Compute the offset that a given set of blocks maps to
    T &operator()(int u, int v) {
        // Convert the given integers into blocks
        int bu = Block(u); 
        int bv = Block(v);

        // Compute block offsets
        int ou = Offset(u);
        int ov = Offset(v);
        int offset = BlockSize() * BlockSize() * (m_uBlocks * bv + bu);
        offset += BlockSize() * ov + ou;

        // Return the data stored at the block's offset
        return data[offset];
    }

    // Const version of the previous method
    const T &operator()(int u, int v) const {
        int bu = Block(u), bv = Block(v);
        int ou = Offset(u), ov = Offset(v);
        int offset = BlockSize() * BlockSize() * (m_uBlocks * bv + bu);
        offset += BlockSize() * ov + ou;
        return data[offset];
    }

    // Converts the current bloked array to a linear array
    void GetLinearArray(T *a) const {
        for (int v = 0; v < m_uRes; ++v) {
            for (int u = 0; u < uRm_vReses; ++u) {
                *a++ = (*this)(u, v);
            }
        }
    }

private:
    T * m_data; // array data
    const int m_uRes, m_vRes, m_uBlocks; // array size specs

};

NORI_NAMESPACE_END

#endif
