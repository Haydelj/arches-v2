#pragma once

namespace DGF {

constexpr size_t MAX_TRIS = 64;
constexpr size_t MAX_VERTS = 64;
constexpr size_t MAX_GEOM_IDS = 32;
constexpr size_t MAX_GEOMID_BITS = 24;
constexpr size_t VERTEX_BIT_ALIGNMENT = 4;
constexpr size_t HEADER_SIZE = 20;

// The 'front buffer' contains the vertex data, OMM palette, and GeomID palettes
constexpr size_t MAX_FRONT_BUFFER_BIT_SIZE = 96 * 8;

// The maximum size of the reuse buffer, after encoding
constexpr size_t MAX_INDEX_BUFFER_BIT_SIZE = 24 * 8;

constexpr size_t MIN_INDEX_BITS = 3;
constexpr size_t MAX_INDEX_BITS = 6;
constexpr size_t MAX_INDICES = 3 * MAX_TRIS;

constexpr size_t MAX_VERTEX_COMPONENT_BITS = 16;
constexpr size_t BLOCK_SIZE = 128;

constexpr int32_t ANCHOR_MIN = ~(0x7fffff);
constexpr int32_t ANCHOR_MAX = 0x7fffff;

constexpr size_t EXPONENT_BIAS = 127;
constexpr size_t EXPONENT_MIN = 1;
constexpr size_t EXPONENT_MAX = 232;

constexpr size_t MAX_GEOMID = ((1 << MAX_GEOMID_BITS) - 1);
constexpr size_t MAX_CONSTANT_GEOMID = ((1 << 9) - 1);
constexpr size_t MAX_PRIMID = ((1 << 29) - 1);

constexpr size_t MAX_OMM_DESCRIPTORS = 7;
constexpr size_t MAX_OMM_HOTPATCH_DWORDS = 2 + MAX_OMM_DESCRIPTORS;

constexpr size_t MAX_USERDATA_SIZE = 4;

struct OffsetVert
{
    uint16_t xyz[3];
};
struct FloatVert
{
    float xyz[3];
};

enum GeomIDModes : uint8_t
{
    GEOMID_CONSTANT,
    GEOMID_PALETTE
};

enum TriControlValues : uint8_t
{
    TC_RESTART = 0,
    TC_EDGE1 = 1,
    TC_EDGE2 = 2,
    TC_BACKTRACK = 3
};

struct MetaData
{
    int32_t  anchorX;
    int32_t  anchorY;
    int32_t  anchorZ;
    uint32_t geomIDPrefix;
    uint32_t primIDBase;

    uint8_t exponent;
    uint8_t xBits;
    uint8_t yBits;
    uint8_t zBits;
    uint8_t numTris;
    uint8_t numVerts;

    GeomIDModes geomIDMode;
    uint8_t numGeomIDs;
    uint8_t geomIDPrefixBitSize;
    uint8_t headerByte;

    uint8_t numOMMDescriptors;
    uint8_t numOMMHotPatchDwords;
    uint8_t haveUserData;
};

static void DecodeMetaData(MetaData* meta, const uint8_t* block);

static size_t DecodeTriangleCount(const uint8_t* block);
static size_t DecodePrimIDBase(const uint8_t* block);
static size_t DecodeVertexCount(const uint8_t* block);

// Decodes triangle indices.. returns number of vertices
static size_t DecodeTriangleList(uint8_t* pIndexBuffer, const uint8_t* block);

// Convert unpacked control values + index buffer into indexed triangle list
static void ConvertTopologyToTriangleList(uint8_t* pIndexBuffer, const TriControlValues* control, const uint8_t* indices, size_t numTris);

// Unpack strip control values and index buffer
//   returns number of vertices
static size_t DecodeTopology(TriControlValues* control, uint8_t* indices, const uint8_t* block);

// Decodes per-triangle OMM descriptor indices
static void DecodeTriangleOMMIndices(uint8_t* pOMMIndices, const uint8_t* block);

// Reads data from the OMM hot-patch section
static size_t ReadOMMHotPatchSection(uint32_t* pHotPatchDwords, const uint8_t* block);

// Decodes per-triangle geomIDs and opacity flags
static void DecodeGeomIDs(uint32_t* pIDs, uint8_t* pOpaqueFlags, const uint8_t* block);

// Convert the block anchor to a floating-point vertex
static FloatVert DecodeAnchor(const MetaData& meta);

// Extract a range of fixed-point vertices from a block
static void DecodeOffsetVerts(size_t numVerts, OffsetVert* pVerts, const uint8_t* block);

// Convert a range of fixed-point vertices to floating-point
static void ConvertOffsetsToFloat(size_t numVerts, FloatVert* pVerts, const OffsetVert* pOffsets, const MetaData& meta);

// Compute the size of an encoded mesh topology
static void ComputeTopologySizes(size_t* pIsFirstBitSize,
    size_t* pControlBitSize,
    size_t* pRepeatIndexBitSize,
    const TriControlValues* control,
    const uint8_t* indices,
    size_t numTris);

// Compute the size of an OMM palette
static size_t ComputeOMMPaletteBitSize(size_t numDescriptors, size_t numTris);

// Reads user-data from an encoded block
static void ReadUserData(const uint8_t* block, void* output, size_t offsetToRead, size_t sizeToRead);

// Inserts user-defined data into a previously encoded block
static void WriteUserData(uint8_t* block, const void* userData, size_t offsetToWrite, size_t sizeToWrite);

struct EncoderInput
{
    int32_t anchorX;        // Fixed-point anchor positions (must fit in 24b)
    int32_t anchorY;
    int32_t anchorZ;
    uint32_t primIDBase;    // Triangle i's primitive ID is: primIDBase+i

    uint8_t exponent;       // Biased exponent for float->Fixed conversion
    uint8_t xBits;          // Number of bits required to encode the offset verts
    uint8_t yBits;
    uint8_t zBits;
    uint8_t numTris;
    uint8_t numVerts;
    uint8_t numIndices;     // Number of entries in the strip index buffer

    uint8_t numOMMDescriptors;
    uint8_t userDataSize;           // User-data size in bytes 
    uint8_t roundTripValidation;

    const OffsetVert* verts;                // Fixed-point, anchor-relative vertex positions
    const TriControlValues* triControl;     // Triangle strip control bits
    const uint8_t* indexBuffer;             // Triangle strip index buffer
    const uint32_t* triangleGeomIDs;
    const uint8_t* triangleOpaqueFlags; // only the LSB is used
    const uint8_t* triangleOMMDescriptorIndices;  // Index of each triangle's descriptor in the OMM palette

};

// Assembles a DGF block from encoding parameters and geometry data
static void Encode(uint8_t* block, const EncoderInput& input);

} // namespace DGF  

#include <bitset>
#include <cassert>
#include <cmath>
#include <tuple>
#include <cstring>

namespace DGF {
struct BlockHeader
{
    // DWORD 0
    uint32_t header_byte : 8;  // must be 0x6
    uint32_t bits_per_index : 2;  // Encodes 3,4,5,6
    uint32_t num_vertices : 6;  // Number of vertices
    uint32_t num_triangles : 6;  // 1-64 (add 1 when decoding)  
    uint32_t geom_id_meta : 10;

    // DWORD 1
    uint32_t exponent : 8;  // Float32 scale (exponent-only) with bias 127.
    // Values 1-232 are supported.
    int32_t  x_anchor : 24;

    // DWORD 2
    uint32_t x_bits : 4;  // 1-16 (XBits + YBits + ZBits must be a multiple of 4 )
    uint32_t y_bits : 4;  // 1-16 (add 1 when decoding for x,y,z bits)
    int32_t y_anchor : 24;

    // DWORD 3
    uint32_t z_bits : 4;  // 1-16
    uint32_t omm_descriptor_count : 3; // 0-7
    uint32_t geom_id_mode : 1;  // 0 = Constant Mode  1= Palette Mode
    int32_t z_anchor : 24;

    // DWORD 4
    uint32_t prim_id_base : 29;
    uint32_t have_user_data : 1;  // If set, first dword following the header is user meta-data.  If clear, first dword contains vertex data
    uint32_t unused : 2; // MBZ

    // 108B of variable-length data segments follow.
};
static_assert(sizeof(BlockHeader) == HEADER_SIZE, "Incorrect structure size");

static uint64_t ReadBits(const uint8_t* bytes, size_t start, size_t len)
{
    assert(len <= 48);

    size_t firstByte = (start) / 8;
    size_t lastByte = (start + (len - 1)) / 8;
    size_t numBytes = 1 + lastByte - firstByte;

    uint64_t dst = 0;
    for(size_t i = 0; i < numBytes; i++)
    {
        uint64_t currByte = bytes[firstByte + i];
        dst |= (currByte << (8 * i));
    }

    return (dst >> (start % 8)) & ((1ull << len) - 1);
}

static void WriteBits(uint8_t* bytes, size_t start, size_t len, uint64_t value)
{
    assert(len <= 48);

    uint64_t mask = ((1ull << len) - 1);
    size_t offset = start % 8;
    value = value << offset;
    mask = mask << offset;

    bytes += (start / 8);
    while(mask)
    {
        size_t currByte = *bytes;
        currByte = (currByte & ~mask) | (value & mask);
        *bytes = (uint8_t)currByte;
        mask = mask >> 8;
        value = value >> 8;
        bytes++;
    }
}

static uint32_t lzcount(uint32_t bits)
{
#ifdef _MSC_VER
    unsigned long out;
    char result = _BitScanReverse(&out, bits);
    return result ? (31 - out) : 32;
#elif __GNUC__
    return bits ? __builtin_clz(bits) : 32;
#else
    static_assert(false, "ScanReverse not ported to this platform.");
#endif
}

static size_t BitsNeededUnsigned(uint32_t bits)
{
    size_t lz = 32 - lzcount(bits);
    return lz;
}

static size_t RoundUpPow2(size_t n, size_t m)
{
    return (n + (m - 1)) & ~(m - 1);
}

static size_t GetOMMIndexSize(size_t numDescriptors)
{
    switch(numDescriptors)
    {
    case 0:
    case 1:
        return 0;
    case 2:
        return 1;
    case 3:
    case 4:
        return 2;
    case 5:
    case 6:
    case 7:
        return 3;
    default:
        assert(false);
        return 0;
    }
}

static size_t GetOMMIndexSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return GetOMMIndexSize(header->omm_descriptor_count);
}

static size_t GetOMMHotPatchSectionDwordSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t haveOMM = header->omm_descriptor_count ? 1 : 0;
    size_t size = 2 * haveOMM + header->omm_descriptor_count;
    return size;
}

static size_t DecodeGeomIDPaletteIDCount(const BlockHeader* header)
{
    return (header->geom_id_meta >> 5) + 1;
}

static size_t DecodeGeomIDPalettePrefixSize(const BlockHeader* header)
{
    return (header->geom_id_meta) & 0x1f;
}

void DecodeMetaData(MetaData* meta, const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    meta->anchorX = header->x_anchor;
    meta->anchorY = header->y_anchor;
    meta->anchorZ = header->z_anchor;
    meta->exponent = header->exponent;
    meta->numTris = header->num_triangles + 1;
    meta->primIDBase = header->prim_id_base;
    meta->xBits = header->x_bits + 1;
    meta->yBits = header->y_bits + 1;
    meta->zBits = header->z_bits + 1;
    meta->headerByte = header->header_byte;
    meta->numVerts = header->num_vertices + 1;
    meta->haveUserData = header->have_user_data;
    if(header->geom_id_mode == 0)
    {
        meta->geomIDMode = GeomIDModes::GEOMID_CONSTANT;
        meta->geomIDPrefix = header->geom_id_meta;
        meta->geomIDPrefixBitSize = 24;
        meta->numGeomIDs = 1;
    }
    else
    {
        size_t numIDs = DecodeGeomIDPaletteIDCount(header);
        size_t prefixBitSize = DecodeGeomIDPalettePrefixSize(header);
        meta->geomIDMode = GeomIDModes::GEOMID_PALETTE;
        meta->numGeomIDs = (uint8_t)numIDs;
        meta->geomIDPrefixBitSize = (uint8_t)prefixBitSize;
        meta->geomIDPrefix = (uint32_t)ReadBits(block + sizeof(BlockHeader), 0, prefixBitSize);
    }

    meta->numOMMDescriptors = header->omm_descriptor_count;
    meta->numOMMHotPatchDwords = (uint8_t)GetOMMHotPatchSectionDwordSize(block);
}

size_t DecodeTriangleCount(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return header->num_triangles + 1;
}
size_t DecodeVertexCount(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return header->num_vertices + 1;
}

size_t DecodePrimIDBase(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return header->prim_id_base;
}

void ConvertTopologyToTriangleList(uint8_t* pIndexBuffer, const TriControlValues* control, const uint8_t* indices, size_t numTris)
{
    // Convert from strip form to list form
    size_t indexPos = 0;
    size_t prev[3] = {0, 0, 0};
    size_t prevPrev[3] = {0, 0, 0};
    for(size_t i = 0; i < numTris; i++)
    {
        DGF::TriControlValues ctrl = control[i];
        size_t v[3] = {0, 0, 0};
        switch(ctrl)
        {
        case TriControlValues::TC_RESTART:
            v[0] = indexPos++;
            v[1] = indexPos++;
            v[2] = indexPos++;
            break;

        case TriControlValues::TC_EDGE1:
            // triangle dangles from second edge (1,2) of predecessor
            v[0] = prev[2];
            v[1] = prev[1];
            v[2] = indexPos++;
            break;

        case TriControlValues::TC_EDGE2:
            // triangle dangles from third edge (2,0) of predecessor
            v[0] = prev[0];
            v[1] = prev[2];
            v[2] = indexPos++;
            break;

        case TriControlValues::TC_BACKTRACK:
            // triangle dangles from opposite edge of predecessor's predecessor
            assert(i >= 2);
            switch(control[i - 1])
            {
            case TriControlValues::TC_EDGE1:
                // triangle dangles from third edge (2,0) of predecessor's predecessor
                v[0] = prevPrev[0];
                v[1] = prevPrev[2];
                v[2] = indexPos++;
                break;

            case TriControlValues::TC_EDGE2:
                // triangle dangles from second edge (1,2) of predecessor
                v[0] = prevPrev[2];
                v[1] = prevPrev[1];
                v[2] = indexPos++;
                break;

            default:
                assert(false);
                break;
            }
        }

        for(size_t j = 0; j < 3; j++)
        {
            pIndexBuffer[3 * i + j] = indices[v[j]];
            prevPrev[j] = prev[j];
            prev[j] = v[j];
        }
    }
}

// Decodes triangle indices.. returns number of vertices
size_t DecodeTriangleList(uint8_t* pIndexBuffer, const uint8_t* block)
{
    // decode control values and count indices
    DGF::TriControlValues controlValues[DGF::MAX_TRIS];
    uint8_t indexBuffer[3 * DGF::MAX_TRIS];

    size_t numVerts = DecodeTopology(controlValues, indexBuffer, block);
    size_t numTris = DecodeTriangleCount(block);
    ConvertTopologyToTriangleList(pIndexBuffer, controlValues, indexBuffer, numTris);
    return numVerts;
}

static size_t DecodeNumVerts(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return header->num_vertices + 1;
}

static size_t DecodeBitsPerVertex(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t xbits = header->x_bits + 1;
    size_t ybits = header->y_bits + 1;
    size_t zbits = header->z_bits + 1;
    return (xbits + ybits + zbits);
}


size_t ComputeOMMPaletteBitSize(size_t numDescriptors, size_t numTris)
{
    if(numDescriptors == 0)
        return 0;

    size_t numHotPatchDwords = 2 + numDescriptors;
    size_t ommIndexSize = GetOMMIndexSize(numTris);
    return 32 * numHotPatchDwords + RoundUpPow2(ommIndexSize * numTris, 8);
}

static size_t GetOMMPaletteBitSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return ComputeOMMPaletteBitSize(header->omm_descriptor_count, header->num_triangles + 1);
}

static size_t GetVertexDataBitSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t numVerts = DecodeNumVerts(block);
    size_t bpv = DecodeBitsPerVertex(block);
    return RoundUpPow2(numVerts * bpv, 8);
}

static size_t GetGeomPaletteBitSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);

    size_t geomIDPaletteBitSize = 0;
    if(header->geom_id_mode)
    {
        size_t numIDs = (header->geom_id_meta >> 5) + 1;
        size_t prefixBitSize = (header->geom_id_meta & 0x1f);
        size_t payloadBitSize = (DGF::MAX_GEOMID_BITS + 1) - prefixBitSize;
        size_t indexBitSize = BitsNeededUnsigned((uint32_t)numIDs - 1);
        size_t numTris = header->num_triangles + 1;

        geomIDPaletteBitSize = RoundUpPow2(numIDs * payloadBitSize + numTris * indexBitSize + prefixBitSize, 8);
    }
    return geomIDPaletteBitSize;
}

static size_t GetFrontBufferBitSize(const uint8_t* block, size_t numVerts)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t vtxBits = header->x_bits + header->y_bits + header->z_bits + 3;
    vtxBits *= numVerts;
    vtxBits = RoundUpPow2(vtxBits, 8);

    return vtxBits + GetGeomPaletteBitSize(block) + GetOMMPaletteBitSize(block);
}

static size_t DecodeBitsPerIndex(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return header->bits_per_index + 3;
}

static size_t GetUserDataBitSize(const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    return 32 * header->have_user_data;
}

static uint8_t* GetUserDataPointer(uint8_t* block)
{
    return block + sizeof(BlockHeader);
}
static const uint8_t* GetUserDataPointer(const uint8_t* block)
{
    return GetUserDataPointer(const_cast<uint8_t*>(block));
}

size_t DecodeTopology(TriControlValues* control, uint8_t* indexBuffer, const uint8_t* block)
{
    size_t numTris = DecodeTriangleCount(block);
    size_t numVerts = DecodeNumVerts(block);

    // decode control values and count indices
    // first triangle is implicitly TC_RESTART, and does not store indices
    control[0] = TriControlValues::TC_RESTART;
    size_t numStoredIndices = 0;
    for(size_t i = 1; i < numTris; i++)
    {
        // remaining triangles reference 3 indices on restart, 1 index otherwise
        size_t ctrl = ReadBits(block, 1024 - 2 * i, 2);

        assert(ctrl == TriControlValues::TC_RESTART ||
            ctrl == TriControlValues::TC_EDGE1 ||
            ctrl == TriControlValues::TC_EDGE2 ||
            ctrl == TriControlValues::TC_BACKTRACK);

        if(ctrl == TriControlValues::TC_RESTART)
            numStoredIndices += 3;
        else
            numStoredIndices++;

        control[i] = static_cast<DGF::TriControlValues>(ctrl);
    }

    // "is-first" bits are immediately after the control bits (back to front)
    size_t isFirstBitPos = 1024 - 2 * (numTris - 1) - 1;

    // index buffer is immediately after the front buffer (front to back)
    size_t indexBitPos = 8 * HEADER_SIZE + GetUserDataBitSize(block) + GetFrontBufferBitSize(block, numVerts);

    // decode the index buffer
    indexBuffer[0] = 0; // first 3 indices are implicit, not stored
    indexBuffer[1] = 1;
    indexBuffer[2] = 2;
    size_t vertexCounter = 3;
    size_t bitsPerIndex = 2;

    bitsPerIndex = DecodeBitsPerIndex(block);

    for(size_t i = 0; i < numStoredIndices; i++)
    {
        bool isFirst = ReadBits(block, isFirstBitPos - i, 1);

        size_t indexValue;
        if(isFirst)
        {
            // first reference to each vertex is omitted
            indexValue = vertexCounter++;
        }
        else
        {
            // second and subsequent refs to each vertex are stored tightly packed
            //  using the minimal number of bits
            indexValue = ReadBits(block, indexBitPos, bitsPerIndex);
            indexBitPos += bitsPerIndex;
        }

        indexBuffer[i + 3] = (uint8_t)indexValue;
    }

    return numVerts;
}

void DecodeGeomIDs(uint32_t* pIDs, uint8_t* pOpaqueFlags, const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t numTris = header->num_triangles + 1;
    assert(numTris <= DGF::MAX_TRIS);

    if(header->geom_id_mode == 0)
    {
        // constant mode
        uint32_t constantID = header->geom_id_meta >> 1;
        uint8_t constantOpaque = (header->geom_id_meta & 1);
        for(size_t i = 0; i < numTris; i++)
        {
            pIDs[i] = constantID;
            pOpaqueFlags[i] = constantOpaque;
        }
    }
    else
    {
        // palette mode
        size_t numIDs = DecodeGeomIDPaletteIDCount(header);
        size_t prefixBitSize = DecodeGeomIDPalettePrefixSize(header);
        size_t payloadBitSize = (DGF::MAX_GEOMID_BITS + 1) - prefixBitSize;
        size_t indexBitSize = BitsNeededUnsigned((uint32_t)numIDs - 1);

        assert(numIDs <= DGF::MAX_GEOM_IDS && numIDs > 0);
        assert(prefixBitSize <= DGF::MAX_GEOMID_BITS);
        assert(payloadBitSize <= DGF::MAX_GEOMID_BITS);

        size_t ommSize = GetOMMPaletteBitSize(block) / 8;
        size_t vertSize = GetVertexDataBitSize(block) / 8;
        size_t userDataSize = GetUserDataBitSize(block) / 8;

        const uint8_t* paletteBits = block + sizeof(BlockHeader) + ommSize + vertSize + userDataSize;

        uint32_t geomIDPrefix = (uint32_t)ReadBits(paletteBits, 0, prefixBitSize);
        geomIDPrefix = geomIDPrefix << payloadBitSize;

        size_t indexOffset = prefixBitSize;
        size_t payloadOffset = indexOffset + numTris * indexBitSize;

        for(size_t i = 0; i < numTris; i++)
        {
            size_t payloadIndex = ReadBits(paletteBits, indexOffset, indexBitSize);
            size_t payloadPos = payloadOffset + payloadIndex * payloadBitSize;
            uint32_t geomIDPayload = (uint32_t)ReadBits(paletteBits, payloadPos, payloadBitSize);
            uint32_t reconstructedID = geomIDPayload | geomIDPrefix;

            pIDs[i] = reconstructedID >> 1;
            pOpaqueFlags[i] = reconstructedID & 1;

            indexOffset += indexBitSize;
        }
    }
}

void DecodeOffsetVerts(size_t numVerts, OffsetVert* pVerts, const uint8_t* block)
{
    assert(numVerts <= DGF::MAX_VERTS);

    size_t userDataSize = GetUserDataBitSize(block);

    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    const uint8_t* vertexData = reinterpret_cast<const uint8_t*>(block) + sizeof(BlockHeader) + userDataSize / 8;

    size_t xbits = header->x_bits + 1;
    size_t ybits = header->y_bits + 1;
    size_t zbits = header->z_bits + 1;

    size_t bpv = xbits + ybits + zbits;
    assert(bpv % DGF::VERTEX_BIT_ALIGNMENT == 0);

    for(size_t i = 0; i < numVerts; i++)
    {
        size_t bitOffset = i * bpv;
        size_t vertex = ReadBits(vertexData, bitOffset, bpv);

        size_t x = vertex;
        size_t y = x >> xbits;
        size_t z = y >> ybits;

        x = x & ((1ull << xbits) - 1);
        y = y & ((1ull << ybits) - 1);
        z = z & ((1ull << zbits) - 1);

        assert(x <= (1 << xbits) - 1);
        assert(y <= (1 << ybits) - 1);
        assert(z <= (1 << zbits) - 1);

        pVerts[i].xyz[0] = static_cast<uint16_t>(x);
        pVerts[i].xyz[1] = static_cast<uint16_t>(y);
        pVerts[i].xyz[2] = static_cast<uint16_t>(z);
    }
}

FloatVert DecodeAnchor(const MetaData& meta)
{
    float scale = std::ldexp(1.0f, meta.exponent - EXPONENT_BIAS);
    return FloatVert{static_cast<float>(meta.anchorX) * scale, static_cast<float>(meta.anchorY) * scale, static_cast<float>(meta.anchorZ) * scale};
}

void ConvertOffsetsToFloat(size_t numVerts, FloatVert* pVerts, const OffsetVert* pOffsets, const MetaData& meta)
{
    assert(numVerts <= DGF::MAX_VERTS);

    float scale = std::ldexp(1.0f, meta.exponent - EXPONENT_BIAS);
    for(size_t i = 0; i < numVerts; i++)
    {
        pVerts[i].xyz[0] = static_cast<float>(pOffsets[i].xyz[0] + meta.anchorX) * scale;
        pVerts[i].xyz[1] = static_cast<float>(pOffsets[i].xyz[1] + meta.anchorY) * scale;
        pVerts[i].xyz[2] = static_cast<float>(pOffsets[i].xyz[2] + meta.anchorZ) * scale;
    }
}

void DecodeTriangleOMMIndices(uint8_t* pOMMIndices, const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t numTris = header->num_triangles + 1;
    size_t ommIndexSize = GetOMMIndexSize(block);
    if(ommIndexSize == 0)
    {
        // no OMM descriptors, fill in zeros
        for(size_t i = 0; i < numTris; i++)
            pOMMIndices[i] = 0;
    }
    else
    {
        // skip the "hot-patch" section and vertex data
        size_t ommPaletteHotPatchSize = 4 * GetOMMHotPatchSectionDwordSize(block);
        size_t vertexDataSize = GetVertexDataBitSize(block) / 8;
        size_t userDataSize = GetUserDataBitSize(block) / 8;

        // read the indices
        const uint8_t* ommIndicesPos = block + sizeof(BlockHeader) + ommPaletteHotPatchSize + vertexDataSize + userDataSize;
        for(size_t i = 0; i < numTris; i++)
            pOMMIndices[i] = (uint8_t)ReadBits(ommIndicesPos, i * ommIndexSize, ommIndexSize);
    }
}

size_t ReadOMMHotPatchSection(uint32_t* pDwords, const uint8_t* block)
{
    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    size_t numDwords = GetOMMHotPatchSectionDwordSize(block);

    size_t vertexDataSize = GetVertexDataBitSize(block) / 8;
    size_t userDataSize = GetUserDataBitSize(block) / 8;

    const uint32_t* hotPatch = reinterpret_cast<const uint32_t*>(block + sizeof(BlockHeader) + vertexDataSize + userDataSize);
    for(size_t i = 0; i < numDwords; i++)
        pDwords[i] = hotPatch[i];
    return numDwords;
}

static void WriteStripControlBits(uint8_t* blockPtr, size_t numTriangles, const DGF::TriControlValues* control)
{
    assert(control[0] == TriControlValues::TC_RESTART);

    control++; // first triangle is implicitly 'RESTART' and not stored
    numTriangles--;

    // write 2-bit pairs into the block from back to front, in reverse order
    blockPtr += DGF::BLOCK_SIZE - 1;
    size_t n4 = 4 * (numTriangles / 4);
    for(size_t i = 0; i < n4; i += 4)
    {
        size_t c0 = control[i + 0];
        size_t c1 = control[i + 1];
        size_t c2 = control[i + 2];
        size_t c3 = control[i + 3];
        size_t ctrl = (c0 << 6) | (c1 << 4) | (c2 << 2) | c3;
        *(blockPtr--) = (uint8_t)ctrl;
    }

    // handle the remainder
    if(numTriangles % 4)
    {
        uint8_t ctrl = 0;
        size_t shift = 6;
        for(size_t i = n4; i < numTriangles; i++)
        {
            size_t c = control[i];
            ctrl |= (c << shift);
            shift -= 2;
        }
        *blockPtr = ctrl;
    }
}

static size_t EncodeTopology(uint8_t* blockPtr, size_t indexBitPos, const EncoderInput& input, size_t bitsPerIndex)
{
    const TriControlValues* control = input.triControl;
    size_t numTriangles = input.numTris;
    const uint8_t* indexBuffer = input.indexBuffer;
    size_t numIndices = input.numIndices;

    if(numTriangles == 0)
        return 0;

    assert(numTriangles <= MAX_TRIS);
    assert(numIndices <= 3 * MAX_TRIS);
    assert(numIndices >= 3);
    assert(indexBuffer[0] == 0 && indexBuffer[1] == 1 && indexBuffer[2] == 2);

    WriteStripControlBits(blockPtr, numTriangles, control);

    // "is-first" bits are immediately after the control bits (back to front)
    size_t isFirstBitPos = 1024 - 2 * (numTriangles - 1) - 1;
    size_t indexStart = indexBitPos;

    static_assert(MAX_VERTS <= 64, "More bits please!");
    uint64_t vertexSeen = 0x7; // first 3 indices are always 0,1,2, and so not stored
    size_t vertexCounter = 3;
    for(size_t i = 3; i < numIndices; i++) // first 3 indices are always 0,1,2, always 'isFirst=1' and so not stored
    {
        size_t idx = indexBuffer[i];
        if(vertexSeen & (1ull << idx))
        {
            // repeated index

            // The 'isFirst' bit is already zero because we zero-initialized the block

            // write the index in the index buffer
            WriteBits(blockPtr, indexBitPos, bitsPerIndex, idx);
            indexBitPos += bitsPerIndex;
        }
        else
        {
            assert(idx == vertexCounter); // vertices must be ordered by first use
            vertexSeen |= (1ull << idx);
            vertexCounter++;

            // write the 'isFirst' bit for this index
            size_t pos = isFirstBitPos - (i - 3);
            blockPtr[pos / 8] |= (1 << (pos % 8));
        }
    }

    size_t indexSize = indexBitPos - indexStart;
    size_t isFirstSize = numIndices - 3;
    size_t controlSize = 2 * (numTriangles - 1);
    size_t topologySize = indexSize + isFirstSize + controlSize;

    assert(indexBitPos <= isFirstBitPos + (numIndices - 3)); // verify that index buffer doesn't collide with 'isFirst' bits
    assert(indexSize <= MAX_INDEX_BUFFER_BIT_SIZE);

    return topologySize;
}

static void EncodeHeader(BlockHeader* header, const EncoderInput& input, size_t bitsPerIndex)
{
    assert(input.anchorX >= ANCHOR_MIN && input.anchorX <= ANCHOR_MAX);
    assert(input.anchorY >= ANCHOR_MIN && input.anchorY <= ANCHOR_MAX);
    assert(input.anchorZ >= ANCHOR_MIN && input.anchorZ <= ANCHOR_MAX);
    assert(input.exponent >= EXPONENT_MIN && input.exponent <= EXPONENT_MAX);
    assert(input.numTris <= MAX_TRIS && input.numTris > 0);
    assert((input.primIDBase + input.numTris - 1) <= MAX_PRIMID);
    assert((input.xBits + input.yBits + input.zBits) % VERTEX_BIT_ALIGNMENT == 0);
    assert(input.numOMMDescriptors <= MAX_OMM_DESCRIPTORS);
    assert(bitsPerIndex <= MAX_INDEX_BITS && bitsPerIndex >= MIN_INDEX_BITS);
    assert(input.numVerts <= MAX_VERTS && input.numVerts > 0);
    assert(input.userDataSize <= MAX_USERDATA_SIZE);

    header->header_byte = 0x6;
    header->x_anchor = input.anchorX;
    header->y_anchor = input.anchorY;
    header->z_anchor = input.anchorZ;
    header->exponent = input.exponent;
    header->prim_id_base = input.primIDBase;
    header->num_triangles = input.numTris - 1;
    header->x_bits = input.xBits - 1;
    header->y_bits = input.yBits - 1;
    header->z_bits = input.zBits - 1;
    header->num_vertices = input.numVerts - 1;
    header->bits_per_index = bitsPerIndex - 3;
    header->omm_descriptor_count = input.numOMMDescriptors;
    header->have_user_data = input.userDataSize ? 1 : 0;

}

static size_t EncodeOMMPalette(uint8_t* block, size_t bitPos, const EncoderInput& input)
{
    if(input.numOMMDescriptors == 0)
        return 0;

    size_t startPos = bitPos;

    // clear hot-patched section
    WriteBits(block, bitPos, 64, 0);
    bitPos += 64;
    size_t descriptorCount = input.numOMMDescriptors;
    for(size_t i = 0; i < descriptorCount; i++)
    {
        WriteBits(block, bitPos, 32, 0);
        bitPos += 32;
    }

    // write pre-computed section
    size_t indexSize = descriptorCount / 2;
    for(size_t i = 0; i < input.numTris; i++)
    {
        size_t idx = input.triangleOMMDescriptorIndices[i];
        assert(idx < (1ull << indexSize));
        WriteBits(block, bitPos, indexSize, idx);
        bitPos += indexSize;
    }

    // clear pad bits
    size_t bitSize = bitPos - startPos;
    if(bitSize % 8)
    {
        size_t numPadBits = 8 - (bitSize % 8);
        WriteBits(block, bitPos, numPadBits, 0);
        bitSize += numPadBits;
    }

    return bitSize;
}

static size_t EncodeGeomIDs(uint8_t* block, size_t bitPos, const EncoderInput& input)
{
    BlockHeader* header = reinterpret_cast<BlockHeader*>(block);

    const uint32_t* triangleGeomIDs = input.triangleGeomIDs;
    size_t numTris = input.numTris;

    uint32_t payloads[MAX_TRIS] = {0};
    uint32_t indices[MAX_TRIS];
    uint32_t numPayloads = 0;

    // deduplicate GeomID/OpaqueFlag pairs.  opaque flag goes in LSB
    for(size_t i = 0; i < numTris; i++)
    {
        uint32_t idx = numPayloads;
        uint32_t geomID = triangleGeomIDs[i];
        uint32_t opaque = input.triangleOpaqueFlags[i];

        assert(geomID <= MAX_GEOMID);

        uint32_t payload = (geomID << 1) + (opaque & 1);
        for(uint32_t j = 0; j < numPayloads; j++)
        {
            if(payloads[j] == payload)
            {
                idx = j;
                break;
            }
        }

        indices[i] = idx;
        if(idx == numPayloads)
            payloads[numPayloads++] = payload;
    }

    assert(numPayloads <= MAX_GEOM_IDS);

    // check if we can use constant-id mode
    if(numPayloads == 1 && (payloads[0] >> 1) <= MAX_CONSTANT_GEOMID)
    {
        header->geom_id_meta = payloads[0];
        header->geom_id_mode = 0;
        return 0;
    }

    // need to use palette mode

    // determine prefix
    uint32_t diff = 0;
    for(size_t i = 1; i < numPayloads; i++)
        diff |= payloads[i] ^ payloads[0];

    size_t prefixSize = lzcount(diff << 7); // force a 25b total size
    size_t payloadSize = 25 - prefixSize;
    size_t indexSize = BitsNeededUnsigned(numPayloads - 1);

    header->geom_id_mode = 1;
    header->geom_id_meta = (prefixSize) | ((numPayloads - 1) << 5);

    // store prefix
    size_t startPos = bitPos;
    WriteBits(block, bitPos, prefixSize, payloads[0] >> payloadSize);
    bitPos += prefixSize;

    // store indices
    if(indexSize > 0)
    {
        for(size_t i = 0; i < numTris; i++)
        {
            WriteBits(block, bitPos, indexSize, indices[i]);
            bitPos += indexSize;
        }
    }

    // store payloads
    if(payloadSize > 0)
    {
        for(size_t i = 0; i < numPayloads; i++)
        {
            WriteBits(block, bitPos, payloadSize, payloads[i]);
            bitPos += payloadSize;
        }
    }

    // clear pad bits
    size_t bitSize = bitPos - startPos;
    if(bitSize % 8)
    {
        size_t numPadBits = 8 - (bitSize % 8);
        WriteBits(block, bitPos, numPadBits, 0);
        bitSize += numPadBits;
    }

    return bitSize;
}

static size_t EncodeVertices(uint8_t* blockData, size_t vertexBitPos, const EncoderInput& input)
{
    assert(input.numVerts * (input.xBits + input.yBits + input.zBits) <= MAX_FRONT_BUFFER_BIT_SIZE);

    size_t startPos = vertexBitPos;
    size_t xbits = input.xBits;
    size_t xybits = input.yBits + input.xBits;
    size_t vertexBits = xybits + input.zBits;
    for(size_t i = 0; i < input.numVerts; i++)
    {
        uint64_t x = input.verts[i].xyz[0];
        uint64_t y = input.verts[i].xyz[1];
        uint64_t z = input.verts[i].xyz[2];
        uint64_t vert = x | (y << xbits) | (z << xybits);
        WriteBits(blockData, vertexBitPos, vertexBits, vert);
        vertexBitPos += vertexBits;
    }

    return RoundUpPow2(vertexBitPos - startPos, 8);
}

void ComputeTopologySizes(size_t* pIsFirstBitSize,
    size_t* pControlBitSize,
    size_t* pRepeatIndexBitSize,
    const TriControlValues* control,
    const uint8_t* indices,
    size_t numTris)
{
    *pControlBitSize = (numTris - 1) * 2; // 2b/tri.  first tri is always 'RESTART'

    size_t numIndices = 0;
    for(size_t i = 0; i < numTris; i++)
        numIndices += (control[i] == TriControlValues::TC_RESTART) ? 3 : 1;

    *pIsFirstBitSize = numIndices - 3; // 1b/index.  first 3 are always 0,1,2

    // compute bits per index and count non-first indices
    size_t numRepeatIndices = 0;
    uint8_t maxRepeatIndex = (1 << MIN_INDEX_BITS) - 1;

    static_assert(MAX_VERTS <= 64, "More bits please!");
    uint64_t VertSeen = 0;
    for(size_t i = 0; i < numIndices; i++)
    {
        uint8_t idx = indices[i];
        if(VertSeen & (1ull << idx))
        {
            maxRepeatIndex = std::max(idx, maxRepeatIndex);
            numRepeatIndices++;
        }
        else
        {
            VertSeen |= (1ull << idx);
        }
    }

    size_t bitsPerIndex = BitsNeededUnsigned(maxRepeatIndex);
    *pRepeatIndexBitSize = bitsPerIndex * numRepeatIndices;
}

void ReadUserData(const uint8_t* block, void* output, size_t offsetToRead, size_t sizeToRead)
{
    size_t userDataSize = GetUserDataBitSize(block) / 8;
    assert(offsetToRead + sizeToRead <= userDataSize);

    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    const uint8_t* userData = GetUserDataPointer(block);
    memcpy(output, userData + offsetToRead, sizeToRead);
}

void WriteUserData(uint8_t* block, const void* input, size_t offsetToWrite, size_t sizeToWrite)
{
    size_t userDataSize = GetUserDataBitSize(block) / 8;
    assert(offsetToWrite + sizeToWrite <= userDataSize);

    const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
    uint8_t* userData = GetUserDataPointer(block);
    memcpy(userData + offsetToWrite, input, sizeToWrite);
}

static size_t ComputeIndexSize(const EncoderInput& input)
{
    static_assert(MAX_VERTS <= 64, "More bits please!");
    uint8_t maxRepeatIndex = (1 << MIN_INDEX_BITS) - 1;
    uint64_t VertSeen = 0;
    for(size_t i = 0; i < input.numIndices; i++)
    {
        uint8_t idx = input.indexBuffer[i];
        if(VertSeen & (1ull << idx))
            maxRepeatIndex = std::max(maxRepeatIndex, idx);
        else
            VertSeen |= (1ull << idx);
    }
    return BitsNeededUnsigned(maxRepeatIndex);
}

void Encode(uint8_t* block, const EncoderInput& input)
{
    memset(block, 0, BLOCK_SIZE);

    size_t bitsPerIndex = ComputeIndexSize(input);

    // header
    BlockHeader* header = reinterpret_cast<BlockHeader*>(block);
    EncodeHeader(header, input, bitsPerIndex);
    size_t headerBitSize = 8 * sizeof(BlockHeader);

    // userdata
    size_t userDataBitSize = input.userDataSize ? 32 : 0;

    // vertex data (optional)
    size_t vertexBitPos = headerBitSize + userDataBitSize;
    size_t vertexBitSize = EncodeVertices(block, vertexBitPos, input);

    // omm palette (optional)
    size_t ommPaletteBitPos = headerBitSize + userDataBitSize + vertexBitSize;
    size_t ommPaletteBitSize = EncodeOMMPalette(block, ommPaletteBitPos, input);

    // geomID palette (optional)
    size_t geomIDBitPos = headerBitSize + userDataBitSize + vertexBitSize + ommPaletteBitSize;
    size_t geomPaletteBitSize = EncodeGeomIDs(block, geomIDBitPos, input);

    // topology
    size_t indexBitPos = headerBitSize + userDataBitSize + vertexBitSize + geomPaletteBitSize + ommPaletteBitSize;
    size_t topologyBitSize = EncodeTopology(block, indexBitPos, input, bitsPerIndex);

    size_t frontBufferBitSize = ommPaletteBitSize + geomPaletteBitSize + vertexBitSize;
    assert(frontBufferBitSize <= DGF::MAX_FRONT_BUFFER_BIT_SIZE);

    assert(headerBitSize + userDataBitSize + frontBufferBitSize + topologyBitSize <= BLOCK_SIZE * 8);

    if(input.roundTripValidation)
    {
        // round-trip verification
        TriControlValues ctrlResult[MAX_TRIS];
        uint8_t indexResult[3 * MAX_TRIS];
        size_t numVerts = DecodeTopology(ctrlResult, indexResult, block);

        assert(numVerts == input.numVerts);

        for(size_t i = 0; i < input.numTris; i++)
            assert(ctrlResult[i] == input.triControl[i]);

        for(size_t i = 0; i < input.numIndices; i++)
        {
            assert(indexResult[i] == input.indexBuffer[i]);
            assert(indexResult[i] < numVerts);
        }

        OffsetVert vertResult[MAX_VERTS];
        DecodeOffsetVerts(input.numVerts, vertResult, block);
        for(size_t i = 0; i < numVerts; i++)
            for(size_t j = 0; j < 3; j++)
                assert(vertResult[i].xyz[j] == input.verts[i].xyz[j]);

        uint32_t geomIDResult[MAX_TRIS];
        uint8_t opaqueFlagResult[MAX_TRIS];
        DecodeGeomIDs(geomIDResult, opaqueFlagResult, block);
        for(size_t i = 0; i < input.numTris; i++)
        {
            assert(geomIDResult[i] == input.triangleGeomIDs[i]);
            assert(opaqueFlagResult[i] == (input.triangleOpaqueFlags[i] & 1));
        }

        uint8_t ommIndices[MAX_TRIS];
        DecodeTriangleOMMIndices(ommIndices, block);
        for(size_t i = 0; i < input.numTris; i++)
        {
            if(input.numOMMDescriptors)
            {
                assert(ommIndices[i] == input.triangleOMMDescriptorIndices[i]);
            }
            else
            {
                assert(ommIndices[i] == 0);
            }
        }
    }
}

} // namespace DGF



#ifndef __riscv
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#endif

#include "triangle.hpp"
#include "bvh.hpp"

//wrapper
namespace rtm {

class DGFMesh
{
public:
    struct Block
    {
        uint8_t data[128];
    };

#ifndef __riscv
    std::vector<Block> blocks;

public:
    DGFMesh(const std::string& file_path)
    {
        bool succeeded = false;
		std::ifstream file_stream(file_path, std::ios::binary | std::ios::ate);
        printf("DFG: Loading: %s\n", file_path.c_str());
        if(file_stream.is_open())
        {
            size_t size = file_stream.tellg();
            blocks.resize(size / sizeof(Block));
            file_stream.seekg(0, std::ios::beg);
            file_stream.read((char*)blocks.data(), size);
            file_stream.close();

            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, blocks[0].data);

            printf("DFG: Loaded\n", file_path.c_str());
            printf("DFG: Exp: %f\n", float32_bf(0, meta.exponent + 16, 0).f32);
            printf("DFG: Size: %.1f MiB\n", (float)sizeof(Block) * blocks.size() / (1 << 20));
        }
        else
        {
            printf("DFG: Failed to load: %s\n", file_path.c_str());
        }
    }

    static uint decompress_block(const DGFMesh::Block& block, IntersectionTriangle* tris)
    {
        DGF::MetaData meta;
        DGF::DecodeMetaData(&meta, block.data);

        uint8_t tmpIndices[DGF::MAX_INDICES];
        DGF::DecodeTriangleList(tmpIndices, block.data);

        uint8_t  opaqueFlags[DGF::MAX_TRIS];
        uint32_t geomID[DGF::MAX_TRIS];
        DGF::DecodeGeomIDs(geomID, opaqueFlags, block.data);

        DGF::OffsetVert offsetVerts[DGF::MAX_VERTS];
        DGF::FloatVert  floatVerts[DGF::MAX_VERTS];
        DGF::DecodeOffsetVerts(meta.numVerts, offsetVerts, block.data);
        DGF::ConvertOffsetsToFloat(meta.numVerts, floatVerts, offsetVerts, meta);

        for(size_t i = 0; i < meta.numTris; i++)
        {
            tris[i].id = geomID[i];
            for(uint j = 0; j < 3; ++j)
                tris[i].tri.vrts[j] = *(rtm::vec3*)floatVerts[tmpIndices[3 * i + j]].xyz;
        }

        return meta.numTris;
    }

#if 0
    BVH2 build_bvh(std::vector<BVH2::BuildObject>& build_objects, bool fast = true)
    {
        build_objects.clear();
        get_build_objects(build_objects);
        BVH2 bvh(build_objects);
        reorder(build_objects);

        if(!fast)
        {
            build_objects.clear();
            uint top_level_nodes = bvh.nodes.size();
            for(uint i = 0; i < top_level_nodes; ++i)
            {
                if(!bvh.nodes[i].data.is_leaf) continue;

                uint root_index = bvh.nodes.size();
                uint block_index = bvh.nodes[i].data.prim_index;
                bvh.nodes[i].data.is_leaf = 0;
                bvh.nodes[i].data.child_index = root_index;

                std::vector<BVH2::BuildObject> temp_build_objects;
                get_build_objects(temp_build_objects, block_index);
                BVH2 block_bvh(temp_build_objects);
                temp_build_objects.clear();

                for(uint j = 0; j < block_bvh.nodes.size(); ++j)
                {
                    BVH2::Node node = block_bvh.nodes[j];
                    if(node.data.is_leaf)
                    {
                        BVH2::BuildObject bobj;
                        bobj.cost = 1.0f;
                        bobj.index = block_index;
                        bobj.aabb = node.aabb;
                        node.data.prim_index = build_objects.size();
                        build_objects.push_back(bobj);
                    }
                    else
                    {
                        node.data.child_index += root_index;
                    }
                    bvh.nodes.push_back(node);
                }
            }
        }

        return bvh;
    }
#endif

    void get_build_objects(std::vector<BVH2::BuildObject>& build_objects)
    {
        for(uint32_t i = 0; i < blocks.size(); ++i)
        {
            IntersectionTriangle tris[DGF::MAX_TRIS];
            uint num_tris = decompress_block(blocks[i], tris);

            {
                std::vector<BVH2::BuildObject> temp_build_objects;
                get_build_objects(temp_build_objects, i);

                std::queue<BVH2::BuildEvent> events;
                events.emplace(0, temp_build_objects.size(), 0);

                //uint split = events.front().split_build_objects_sah(temp_build_objects.data());
                //if(split != ~0u)
                //{
                //    events.emplace(events.front().start, split, 0);
                //    events.emplace(split, events.front().end, split, 0);
                //    events.pop();
                //}
                //else
                //{
                //    events.push(events.front());
                //    events.pop();
                //}

                while(!events.empty())
                {
                    BVH2::BuildObject build_object;
                    build_object.cost = 1.0;
                    build_object.index = i;
                    build_object.aabb = AABB();
                    for(uint j = events.front().start; j < events.front().end; ++j)
                        build_object.aabb.add(tris[j].tri.aabb());
                    build_objects.push_back(build_object);
                    events.pop();
                }
            }
        }
    }

    void get_build_objects(std::vector<BVH2::BuildObject>& build_objects, uint i)
    {
        IntersectionTriangle tris[DGF::MAX_TRIS];
        uint num_tris = decompress_block(blocks[i], tris);

        for(uint j = 0; j < num_tris; ++j)
        {
            BVH2::BuildObject build_object;
            build_object.cost = 1.0;
            build_object.index = i;
            build_object.aabb = tris[j].tri.aabb();
            build_objects.push_back(build_object);
        }
    }

    void reorder(std::vector<BVH2::BuildObject>& ordered_build_objects)
    {
        std::vector<Block> tmp_blocks(blocks);
        blocks.resize(ordered_build_objects.size());
        for(uint32_t i = 0; i < ordered_build_objects.size(); ++i)
        {
            blocks[i] = tmp_blocks[ordered_build_objects[i].index];
            ordered_build_objects[i].index = i;
        }
    }
#endif
};

#ifndef __riscv
inline uint decompress(const DGFMesh::Block& block, uint strip_id, IntersectionTriangle* tris)
{
    return rtm::DGFMesh::decompress_block(block, tris);
}
#endif

}
