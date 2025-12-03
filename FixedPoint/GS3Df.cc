/*
 * ============================================================================
 *                              GS3Df.CC - Fixed32 Optimized Version
 * ============================================================================
 * 
 * 3D rendering program for Apple IIGS with ORCA/C - FIXED POINT VERSION
 * Version 0.4 Fixed-Point 64-bit - Ultra-Optimized Fixed32 Arithmetic
 * 
 * DESCRIPTION:
 *   This program reads 3D model files in simplified OBJ format,
 *   applies 3D geometric transformations (rotation, translation),
 *   projects the points on a 2D screen, and draws the resulting polygons
 *   using QuickDraw.
 *   
 *   This is the OPTIMIZED VERSION using Fixed32 16.16 arithmetic with
 *   lookup tables and 64-bit overflow protection. 
 *   Significantly faster performance compared to SANE Extended reference version.
 * 
 * FEATURES:
 *   - Reading OBJ files (vertices "v" and faces "f")
 *   - 3D transformations using Fixed32 16.16 arithmetic
 *   - Degree-to-radian conversion via lookup table (361 entries)
 *   - 64-bit arithmetic for overflow-safe multiplication/division
 *   - Perspective projection on 2D screen
 *   - Graphic rendering with QuickDraw
 *   - Interactive interface with corrected parameter display
 *   - Performance measurements for comparison with SANE version
 * 
 * ARITHMETIC:
 *   - Fixed32 16.16 format (16 bits integer + 16 bits fractional)
 *   - FIXED_SCALE = 65536, optimized FIXED_MUL_64/FIXED_DIV_64
 *   - Pre-computed trigonometric lookup tables
 *   - Overflow-safe 64-bit intermediate calculations
 * 
 * PERFORMANCE OPTIMIZATIONS:
 *   - Direct lookup table access: rad = deg_to_rad_table[degree]
 *   - Combined transformation+projection pipeline
 *   - Pre-calculated trigonometric products
 *   - Eliminated function calls in critical loops
 * 
 * AUTHOR: Bruno
 * DATE: 2025
 * PLATFORM: Apple IIGS - ORCA/C with Fixed32 arithmetic
 * ============================================================================
 */

// ============================================================================
//                           HEADER INCLUDES
// ============================================================================

#include <stdio.h>      // Standard input/output (printf, fgets, etc.)
#include <asm.h>        // ORCA specific assembler functions
#include <string.h>     // String manipulation (strlen, strcmp, etc.)
#include <misctool.h>   // ORCA misc tools (GetTick, keypress, etc.)
#include <stdlib.h>     // Standard functions (malloc, free, atof, etc.)
#include <math.h>       // Math functions (cos, sin, sqrt, etc.)
#include <quickdraw.h>  // Apple IIGS QuickDraw graphics API
#include <event.h>      // System event management
#include <memory.h>     // Advanced memory management (NewHandle, etc.)
#include <window.h>     // Window management
#include <orca.h>       // ORCA specific functions (startgraph, etc.)

// --- Variable globale pour la vérification des indices de sommet dans readFaces ---
// (Ajouté pour garantir la cohérence OBJ)
int readVertices_last_count = 0;

// --- Global persistent polygon handle for drawing ---
// Allocated once, HLocked only during use, prevents repeated NewHandle/DisposeHandle
static Handle globalPolyHandle = NULL;
static int poly_handle_locked = 0;  // Track lock state

// ============================================================================
//                            FIXED POINT DEFINITIONS
// ============================================================================

/**
 * FIXED POINT ARITHMETIC - 64-bit safe version
 * =============================================
 * 
 * Cette version utilise l'arithmétique en virgule fixe 16.16 avec
 * des calculs intermédiaires 64-bit pour éviter les débordements.
 * 
 * Format: 16.16 (16 bits entiers, 16 bits fractionnaires)
 * Plage: -32768.0 à +32767.99998 avec précision de 1/65536
 */

// Basic fixed-point definitions
typedef long Fixed32;           // 32-bit fixed point number (16.16)
typedef long long Fixed64;      // 64-bit for intermediate calculations

#define FIXED_SHIFT     16                    // Number of fractional bits
#define FIXED_SCALE     (1L << FIXED_SHIFT)  // 65536
#define FIXED_MASK      (FIXED_SCALE - 1)    // 0xFFFF
#define FIXED_HALF      (FIXED_SCALE >> 1)   // 32768 (for rounding)

// Mathematical constants in fixed point
#define FIXED_PI        205887L               // PI ≈ 3.14159265 in 16.16
#define FIXED_2PI       411775L               // 2*PI ≈ 6.28318530 in 16.16
#define FIXED_PI_2      102944L               // PI/2 ≈ 1.57079632 in 16.16
#define FIXED_ONE       FIXED_SCALE           // 1.0 in 16.16
#define FIXED_PI_180    1143LL                // PI/180 ≈ 0.017453293 in 16.16 (64-bit)

// Lookup table: degrees to radians in Fixed32 (0° to 360°)
// Ultra-fast conversion - avoids all multiplication/division
static const Fixed32 deg_to_rad_table[361] = {
    0,      1143,   2287,   3430,   4573,   5717,   6860,   8003,   9147,   10290,  // 0-9°
    11433,  12577,  13720,  14863,  16007,  17150,  18293,  19437,  20580,  21723,  // 10-19°
    22867,  24010,  25153,  26297,  27440,  28583,  29727,  30870,  32013,  33157,  // 20-29°
    34300,  35443,  36587,  37730,  38873,  40017,  41160,  42303,  43447,  44590,  // 30-39°
    45733,  46877,  48020,  49163,  50307,  51450,  52593,  53737,  54880,  56023,  // 40-49°
    57167,  58310,  59453,  60597,  61740,  62883,  64027,  65170,  66313,  67457,  // 50-59°
    68600,  69743,  70887,  72030,  73173,  74317,  75460,  76603,  77747,  78890,  // 60-69°
    80033,  81177,  82320,  83463,  84607,  85750,  86893,  88037,  89180,  90323,  // 70-79°
    91467,  92610,  93753,  94897,  96040,  97183,  98327,  99470, 100613, 101757,  // 80-89°
    102900,104043, 105187, 106330, 107473, 108617, 109760, 110903, 112047, 113190,  // 90-99°
    114333,115477, 116620, 117763, 118907, 120050, 121193, 122337, 123480, 124623,  // 100-109°
    125767,126910, 128053, 129197, 130340, 131483, 132627, 133770, 134913, 136057,  // 110-119°
    137200,138343, 139487, 140630, 141773, 142917, 144060, 145203, 146347, 147490,  // 120-129°
    148633,149777, 150920, 152063, 153207, 154350, 155493, 156637, 157780, 158923,  // 130-139°
    160067,161210, 162353, 163497, 164640, 165783, 166927, 168070, 169213, 170357,  // 140-149°
    171500,172643, 173787, 174930, 176073, 177217, 178360, 179503, 180647, 181790,  // 150-159°
    182933,184077, 185220, 186363, 187507, 188650, 189793, 190937, 192080, 193223,  // 160-169°
    194367,195510, 196653, 197797, 198940, 200083, 201227, 202370, 203513, 204657,  // 170-179°
    205800,206943, 208087, 209230, 210373, 211517, 212660, 213803, 214947, 216090,  // 180-189°
    217233,218377, 219520, 220663, 221807, 222950, 224093, 225237, 226380, 227523,  // 190-199°
    228667,229810, 230953, 232097, 233240, 234383, 235527, 236670, 237813, 238957,  // 200-209°
    240100,241243, 242387, 243530, 244673, 245817, 246960, 248103, 249247, 250390,  // 210-219°
    251533,252677, 253820, 254963, 256107, 257250, 258393, 259537, 260680, 261823,  // 220-229°
    262967,264110, 265253, 266397, 267540, 268683, 269827, 270970, 272113, 273257,  // 230-239°
    274400,275543, 276687, 277830, 278973, 280117, 281260, 282403, 283547, 284690,  // 240-249°
    285833,286977, 288120, 289263, 290407, 291550, 292693, 293837, 294980, 296123,  // 250-259°
    297267,298410, 299553, 300697, 301840, 302983, 304127, 305270, 306413, 307557,  // 260-269°
    308700,309843, 310987, 312130, 313273, 314417, 315560, 316703, 317847, 318990,  // 270-279°
    320133,321277, 322420, 323563, 324707, 325850, 326993, 328137, 329280, 330423,  // 280-289°
    331567,332710, 333853, 334997, 336140, 337283, 338427, 339570, 340713, 341857,  // 290-299°
    343000,344143, 345287, 346430, 347573, 348717, 349860, 351003, 352147, 353290,  // 300-309°
    354433,355577, 356720, 357863, 359007, 360150, 361293, 362437, 363580, 364723,  // 310-319°
    365867,367010, 368153, 369297, 370440, 371583, 372727, 373870, 375013, 376157,  // 320-329°
    377300,378443, 379587, 380730, 381873, 383017, 384160, 385303, 386447, 387590,  // 330-339°
    388733,389877, 391020, 392163, 393307, 394450, 395593, 396737, 397880, 399023,  // 340-349°
    400167,401310, 402453, 403597, 404740, 405883, 407027, 408170, 409313, 410457,  // 350-359°
    411600  // 360°
};

// Fast degree to radian conversion using lookup table


// Conversion macros
#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)

// Arithmetic operations
#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))
#define FIXED_ABS(x)        ((x) >= 0 ? (x) : -(x))
#define FIXED_FRAC(x)       ((x) & FIXED_MASK)

// Simple multiplication and division for ORCA/C
#define FIXED_MUL(a, b)     (((long)(a) * (long)(b)) >> FIXED_SHIFT)
#define FIXED_DIV(a, b)     (((long)(a) << FIXED_SHIFT) / (long)(b))

// 64-bit safe multiplication and division for critical calculations
#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))
#define FIXED64_TO_32(x)    ((Fixed32)(x))

// ============================================================================
//                            GLOBAL CONSTANTS
// ============================================================================

// Performance and debug configuration
#define ENABLE_DEBUG_SAVE 0     // 1 = Enable debug save (SLOW!), 0 = Disable
//#define PERFORMANCE_MODE 0      // 1 = Optimized performance mode, 0 = Debug mode
// OPTIMIZATION: Performance mode - disable printf
#define PERFORMANCE_MODE 1      // 1 = no printf, 0 = normal printf

#define MAX_LINE_LENGTH 256     // Maximum file line size
#define MAX_VERTICES 6000       // Maximum vertices in a 3D model
#define MAX_FACES 6000          // Maximum faces in a 3D model (using parallel arrays)
#define MAX_FACE_VERTICES 6     // Maximum vertices per face (triangles/quads/hexagons)
#define PI 3.14159265359        // Mathematical constant Pi
#define CENTRE_X 160            // Screen center in X (320/2)
#define CENTRE_Y 100            // Screen center in Y (200/2)
//#define mode 640               // Graphics mode 640x200 pixels
#define mode 320               // Graphics mode 320x200 pixels

// ============================================================================
//                          DATA STRUCTURES
// ============================================================================

/**
 * Structure Vertex3D
 * 
 * DESCRIPTION:
 *   Represents a point in 3D space with its different representations
 *   throughout the 3D rendering pipeline.
 * 
 * FIELDS:
 *   x, y, z    : Original coordinates read from OBJ file
 *   xo, yo, zo : Transformed coordinates in the observer system
 *                (after applying rotations and translation)
 *   x2d, y2d   : Final projected coordinates on 2D screen
 * 
 * USAGE:
 *   This structure preserves all transformation steps to
 *   allow debugging and recalculations without rereading the file.
 */


// Parallel arrays for vertex data (to break 32K/64K struct limit)
typedef struct {
    Handle xHandle, yHandle, zHandle;
    Handle xoHandle, yoHandle, zoHandle;
    Handle x2dHandle, y2dHandle;
    Fixed32 *x, *y, *z;
    Fixed32 *xo, *yo, *zo;
    int *x2d, *y2d;
    int vertex_count;
} VertexArrays3D;

/**
 * Structure FaceArrays3D - Compact dynamic face storage with depth-sorted rendering
 * Each face stores ONLY the vertices it needs:
 * - vertex_count: How many vertices this face has (3 for tri, 4 for quad, etc.)
 * - vertex_indices_buffer: ONE packed buffer with all indices (NO WASTED SLOTS!)
 * - vertex_indices_ptr: Offset array pointing to each face's slice in the buffer
 * - sorted_face_indices: Array of face indices SORTED by depth (for painter's algorithm)
 * - z_max: Depth for sorting
 * - display_flag: Culling flag
 * 
 * MEMORY LAYOUT:
 * Instead of 4 arrays of 6000 elements each, we use ONE packed buffer.
 * Triangles (1538 faces × 3 indices) + Quads (2504 faces × 4 indices) = packed linearly
 * Saves ~40-60% memory vs fixed 4 vertices/face
 * 
 * DEPTH SORTING STRATEGY:
 * Instead of moving data around (complex with variable-length indices), we maintain
 * sorted_face_indices[] which contains face numbers in depth order (farthest first).
 * Drawing loop: for(i=0; i<face_count; i++) { int face_id = sorted_face_indices[i]; ... }
 * This keeps the buffer untouched while providing correct rendering order.
 */
typedef struct {
    Handle vertex_countHandle;           // 1 array: face_count × 4 bytes
    Handle vertex_indicesBufferHandle;   // 1 buffer: all indices packed (NO WASTED SLOTS!)
    Handle vertex_indicesPtrHandle;      // 1 array: offset to each face's indices
    Handle z_maxHandle;                  // 1 array: face_count × 4 bytes
    Handle display_flagHandle;           // 1 array: face_count × 4 bytes
    Handle sorted_face_indicesHandle;    // 1 array: face numbers sorted by depth
    
    int *vertex_count;                   // Points to: [3, 3, 4, 3, 4, ...]
    int *vertex_indices_buffer;          // Points to: [v1, v2, v3, v1, v2, v3, v4, v1, v2, v3, ...]
    int *vertex_indices_ptr;             // Points to: [offset0, offset3, offset6, offset10, ...]
    Fixed32 *z_max;
    int *display_flag;
    int *sorted_face_indices;            // Points to: [face_id1, face_id2, ...] sorted by z_max
    int face_count;                      // Actual number of loaded faces
    int total_indices;                   // Total indices across all faces (sum of all vertex_counts)
} FaceArrays3D;

/**
 * Structure Face3D
 * 
 * DESCRIPTION:
 *   Represents a face (polygon) of a 3D object. A face is defined
 *   by a list of indices pointing to vertices in the model's
 *   vertex array.
 * 
 * FIELDS:
 *   vertex_count    : Number of vertices composing this face (3+ for polygon)
 *   vertex_indices  : Array of vertex indices (1-based numbering as in OBJ format)
 * 
 * NOTES:
 *   - Indices are stored in base 1 (first vertex = index 1)
 *   - Conversion to base 0 needed to access the C array
 *   - Maximum MAX_FACE_VERTICES vertices per face (now 6 for triangles/quads/hexagons)
 *   - LEGACY STRUCTURE: Now replaced by FaceArrays3D for parallel array storage
 */
typedef struct {
    int vertex_count;                           // Number of vertices in the face
    int vertex_indices[MAX_FACE_VERTICES];     // Vertex indices (base 1, max 6 for polygons)
    Fixed32 z_max;                             // Maximum depth of the face (for sorting, Fixed Point)
    int display_flag;                          // 1 = display face, 0 = don't display (behind camera)
} Face3D;

/**
 * Structure DynamicPolygon
 * 
 * DESCRIPTION:
 *   Structure compatible with QuickDraw for drawing polygons.
 *   This structure must be dynamically allocated because its size
 *   varies according to the number of points in the polygon.
 * 
 * FIELDS:
 *   polySize    : Total size of the structure in bytes
 *   polyBBox    : Polygon bounding box rectangle
 *   polyPoints  : Array of polygon points in screen coordinates
 * 
 * QUICKDRAW FORMAT:
 *   QuickDraw expects a structure with header (size + bbox) followed
 *   by points. The size must include the header + all points.
 */
typedef struct {
    int polySize;                               // Total structure size (bytes)
    Rect polyBBox;                             // Bounding box rectangle
    Point polyPoints[MAX_FACE_VERTICES];       // Polygon points (screen coordinates)
} DynamicPolygon;

/**
 * Structure ObserverParams
 * 
 * DESCRIPTION:
 *   Contains all parameters defining the position and orientation
 *   of the observer (camera) in 3D space, as well as projection
 *   parameters.
 * 
 * FIELDS:
 *   angle_h  : Horizontal rotation angle of the observer (degrees)
 *              Rotation around Y-axis (left/right)
 *   angle_v  : Vertical rotation angle of the observer (degrees)
 *              Rotation around X-axis (up/down)
 *   angle_w  : Screen projection rotation angle (degrees)
 *              Rotation in the final 2D plane
 *   distance : Distance from observer to model center
 *              Larger = smaller object, smaller = larger object
 * 
 * MATHEMATICAL NOTES:
 *   - Angles are in degrees (converted to radians for calculations)
 *   - Distance affects perspective and apparent size
 *   - angle_w allows final rotation to adjust orientation
 */
typedef struct {
    Fixed32 angle_h;   // Observer horizontal angle (Y rotation, Fixed Point)
    Fixed32 angle_v;   // Observer vertical angle (X rotation, Fixed Point) 
    Fixed32 angle_w;   // 2D projection rotation angle (Fixed Point)
    Fixed32 distance;  // Observer-object distance (perspective, Fixed Point)
} ObserverParams;

/**
 * Structure Model3D
 * 
 * DESCRIPTION:
 *   Main structure containing all data of a 3D model.
 *   It groups vertices, faces, and associated counters.
 * 
 * FIELDS:
 *   vertices      : Pointer to dynamic vertex array
 *   faces         : Pointer to dynamic face array
 *   vertex_count  : Actual number of loaded vertices
 *   face_count    : Actual number of loaded faces
 * 
 * MEMORY MANAGEMENT:
 *   - Arrays are dynamically allocated (malloc)
 *   - Allows exceeding Apple IIGS stack limits
 *   - Mandatory cleanup with destroyModel3D()
 * 
 * USAGE:
 *   Model3D* model = createModel3D();
 *   loadModel3D(model, "file.obj");
 *   // ... usage ...
 *   destroyModel3D(model);
 */
typedef struct {
    VertexArrays3D vertices;          // Parallel arrays for all vertex data
    FaceArrays3D faces;               // Parallel arrays for all face data
} Model3D;

// ============================================================================
//                       FUNCTION DECLARATIONS
// ============================================================================

/**
 * FIXED POINT MATHEMATICAL FUNCTIONS
 * ===================================
 */

/**
 * Fixed-point trigonometric functions
 * Uses Taylor series approximation for sin/cos
 */
Fixed32 sin_fixed(Fixed32 angle);
Fixed32 cos_fixed(Fixed32 angle);


/**
 * OBJ FILE READING FUNCTIONS
 * ===========================
 */

/**
 * readVertices
 * 
 * DESCRIPTION:
 *   Reads vertices (3D points) from an OBJ format file.
 *   Searches for lines starting with "v " and extracts X,Y,Z coordinates.
 * 
 * PARAMETERS:
 *   filename     : OBJ filename to read
 *   vertices     : Destination array to store vertices
 *   max_vertices : Maximum array size (overflow protection)
 * 
 * RETURN:
 *   Number of successfully read vertices, or -1 on error
 * 
 * OBJ FORMAT:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14
 */
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices);

/**
 * readFaces
 * 
 * DESCRIPTION:
 *   Reads faces (polygons) from an OBJ format file.
 *   Searches for lines starting with "f " and extracts vertex indices.
 * 
 * PARAMETERS:
 *   filename   : OBJ filename to read
 *   faces      : Destination array to store faces
 *   max_faces  : Maximum array size (overflow protection)
 * 
 * RETURN:
 *   Number of successfully read faces, or -1 on error
 * 
 * OBJ FORMAT:
 *   f 1 2 3        (triangle with vertices 1, 2, 3)
 *   f 4 5 6 7      (quadrilateral with vertices 4, 5, 6, 7)
 */

/**
 * 3D GEOMETRIC TRANSFORMATION FUNCTIONS
 * =====================================
 */

/**
 * transformToObserver
 * 
 * DESCRIPTION:
 *   Applies 3D geometric transformations to go from the model's
 *   coordinate system to the observer's coordinate system.
 *   
 *   APPLIED TRANSFORMATIONS:
 *   1. Horizontal rotation (angle_h) around Y-axis
 *   2. Vertical rotation (angle_v) around X-axis
 *   3. Translation by observation distance
 * 
 * PARAMETERS:
 *   vertices     : Array of vertices to transform
 *   vertex_count : Number of vertices in the array
 *   angle_h      : Horizontal rotation angle (degrees)
 *   angle_v      : Vertical rotation angle (degrees)
 *   distance     : Observation distance (Z translation)
 * 
 * MATHEMATICAL FORMULAS:
 *   zo = -x*(cos_h*cos_v) - y*(sin_h*cos_v) - z*sin_v + distance
 *   xo = -x*sin_h + y*cos_h
 *   yo = -x*(cos_h*sin_v) - y*(sin_h*sin_v) + z*cos_v
 * 
 * RESULTING COORDINATES:
 *   The xo, yo, zo fields of the vertices are updated.
 */
void transformToObserver(VertexArrays3D* vtx, Fixed32 angle_h, Fixed32 angle_v, Fixed32 distance);

/**
 * projectTo2D
 * 
 * DESCRIPTION:
 *   Projects the transformed 3D coordinates onto a 2D screen using
 *   perspective projection. Also applies a final rotation
 *   in the 2D plane.
 * 
 * PARAMETERS:
 *   vertices     : Array of vertices to project
 *   vertex_count : Number of vertices in the array
 *   angle_w      : Rotation angle in the 2D plane (degrees)
 * 
 * ALGORITHM:
 *   1. Perspective projection: x2d = (xo * scale) / zo + center_x
 *   2. Same for y2d with Y-axis inversion
 *   3. Final rotation in the 2D plane according to angle_w
 *   4. Points behind observer (zo <= 0) marked invisible
 * 
 * RESULTING COORDINATES:
 *   The x2d, y2d fields of the vertices contain the final screen coordinates.
 */
void projectTo2D(VertexArrays3D* vtx, Fixed32 angle_w);

/**
 * GRAPHIC RENDERING FUNCTIONS
 * ===========================
 */

/**
 * drawPolygons
 * 
 * DESCRIPTION:
 *   Draws all polygons (faces) of the 3D model on screen using
 *   Apple IIGS QuickDraw API. Each face is rendered with a different
 *   color for visualization.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with calculated 2D coordinates
 *   faces      : Array of faces to draw
 *   face_count : Number of faces in the array
 * 
 * ALGORITHM:
 *   1. QuickDraw graphics mode initialization
 *   2. For each face:
 *      - Check vertex visibility
 *      - Create QuickDraw polygon structure
 *      - Calculate bounding box
 *      - Dynamic memory allocation
 *      - Drawing with PaintPoly()
 *      - Memory cleanup
 * 
 * COLOR MANAGEMENT:
 *   Cyclic colors based on face index (i % 15 + 1)
 * 
 * OPTIMIZATIONS:
 *   - Faces with less than 3 visible vertices ignored
 *   - Off-screen vertices handled correctly
 */
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total);
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count);
void sortFacesByDepth(Model3D* model, int face_count);
void sortFacesByDepth_insertion(FaceArrays3D* faces, int face_count);
void sortFacesByDepth_insertion_range(FaceArrays3D* faces, int low, int high);
void sortFacesByDepth_quicksort(FaceArrays3D* faces, int low, int high);
int sortFacesByDepth_partition(FaceArrays3D* faces, int low, int high);
int partition_median3(Face3D* faces, int low, int high);

// Function to save debug data to disk
void saveDebugData(Model3D* model, const char* debug_filename);

/**
 * UTILITY FUNCTIONS
 * ==================
 */

/**
 * delay
 * 
 * DESCRIPTION:
 *   Wait a specific number of seconds using the Apple IIGS
 *   system timer.
 * 
 * PARAMETERS:
 *   seconds : Number of seconds to wait
 * 
 * IMPLEMENTATION:
 *   Uses GetTick() which returns the number of ticks since startup
 *   (60 ticks per second on Apple IIGS).
 */
void delay(int seconds);

/**
 * 3D MODEL MANAGEMENT FUNCTIONS
 * ==============================
 */

/**
 * createModel3D
 * 
 * DESCRIPTION:
 *   Creates and initializes a new Model3D structure with dynamic
 *   allocation of vertex and face arrays.
 * 
 * RETURN:
 *   Pointer to the new structure, or NULL on memory error
 * 
 * MEMORY MANAGEMENT:
 *   - Main structure allocation
 *   - Vertex array allocation (MAX_VERTICES)
 *   - Face array allocation (MAX_FACES)
 *   - Automatic cleanup on partial failure
 */
Model3D* createModel3D(void);

/**
 * destroyModel3D
 * 
 * DESCRIPTION:
 *   Frees all memory associated with a 3D model.
 * 
 * PARAMETERS:
 *   model : Pointer to the model to destroy (can be NULL)
 * 
 * CLEANUP:
 *   - Vertex array
 *   - Face array  
 *   - Main structure
 */
void destroyModel3D(Model3D* model);

/**
 * loadModel3D
 * 
 * DESCRIPTION:
 *   Loads a complete 3D model from an OBJ file.
 *   Combines reading vertices and faces.
 * 
 * PARAMETERS:
 *   model    : Destination Model3D structure
 *   filename : OBJ filename to load
 * 
 * RETURN:
 *   0 on success, -1 on error
 * 
 * PROCESSING:
 *   - Read vertices with readVertices()
 *   - Read faces with readFaces()
 *   - Update counters in the structure
 */
int loadModel3D(Model3D* model, const char* filename);

/**
 * USER INTERFACE FUNCTIONS
 * =========================
 */

/**
 * getObserverParams
 * 
 * DESCRIPTION:
 *   User interface for entering observer parameters
 *   (angles, distance, etc.).
 * 
 * PARAMETERS:
 *   params : Destination structure for parameters
 * 
 * INTERFACE:
 *   Interactive prompts to user:
 *   - Horizontal angle
 *   - Vertical angle  
 *   - Observation distance
 *   - Screen rotation angle
 */
void getObserverParams(ObserverParams* params);

/**
 * displayModelInfo
 * 
 * DESCRIPTION:
 *   Displays general information about the loaded model
 *   (number of vertices, number of faces).
 * 
 * PARAMETERS:
 *   model : Model whose information to display
 */
void displayModelInfo(Model3D* model);

/**
 * displayResults
 * 
 * DESCRIPTION:
 *   Displays detailed results of the rendering pipeline:
 *   original, transformed, and projected coordinates.
 *   Also launches graphic rendering.
 * 
 * PARAMETERS:
 *   model : Model whose results to display
 */
void displayResults(Model3D* model);

/**
 * processModelFast
 * 
 * DESCRIPTION:
 *   Executes the complete 3D transformation pipeline on a model:
 *   transformation to observer then 2D projection in one optimized pass.
 * 
 * PARAMETERS:
 *   model  : Model to process
 *   params : Transformation parameters
 */
void processModelFast(Model3D* model, ObserverParams* params, const char* filename);

// ============================================================================
//                          FUNCTION IMPLEMENTATIONS
// ============================================================================

/**
 * UTILITY FUNCTION: delay
 * ========================
 * 
 * This function uses the Apple IIGS system timer to implement
 * precise waiting. GetTick() returns the number of ticks elapsed
 * since system startup (60 Hz on Apple IIGS).
 */
void delay(int seconds) {
    long startTick = GetTick();               // Start time (in ticks)
    long ticksToWait = seconds * 60;          // Conversion: 60 ticks/second
    
    // Active wait loop until delay elapsed
    while (GetTick() - startTick < ticksToWait) {
        // Passive wait - processor remains available for system
    }
}

// ============================================================================
//                    FIXED POINT MATHEMATICAL FUNCTIONS
// ============================================================================

/**
 * SINE FUNCTION - Fixed Point Implementation
 * =========================================== 
 * 
 * Calculates sin(x) using Taylor series approximation.
 * Input: angle in radians (fixed point 16.16)
 * Output: sin(angle) in fixed point 16.16
 * 
 * Taylor series: sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + ...
 * Optimized for range [-2π, 2π] with angle normalization.
 */
Fixed32 sin_fixed(Fixed32 angle) {
    // Normalize angle to [-PI, PI] range
    while (angle > FIXED_PI) angle = FIXED_SUB(angle, FIXED_2PI);
    while (angle < -FIXED_PI) angle = FIXED_ADD(angle, FIXED_2PI);
    
    Fixed32 x = angle;
    Fixed32 x2 = FIXED_MUL_64(x, x);       // x²
    Fixed32 x3 = FIXED_MUL_64(x2, x);      // x³
    Fixed32 x5 = FIXED_MUL_64(x3, x2);     // x⁵
    Fixed32 x7 = FIXED_MUL_64(x5, x2);     // x⁷
    
    // Taylor series: sin(x) = x - x³/6 + x⁵/120 - x⁷/5040
    Fixed32 result = x;
    result = FIXED_SUB(result, FIXED_DIV_64(x3, INT_TO_FIXED(6)));      // -x³/3!
    result = FIXED_ADD(result, FIXED_DIV_64(x5, INT_TO_FIXED(120)));     // +x⁵/5!
    result = FIXED_SUB(result, FIXED_DIV_64(x7, INT_TO_FIXED(5040)));    // -x⁷/7!
    
    return result;
}

/**
 * COSINE FUNCTION - Fixed Point Implementation
 * ============================================
 * 
 * Calculates cos(x) using the identity: cos(x) = sin(x + π/2)
 * More efficient than separate Taylor series.
 */
Fixed32 cos_fixed(Fixed32 angle) {
    return sin_fixed(FIXED_ADD(angle, FIXED_PI_2));
}

// ============================================================================
//                    3D MODEL MANAGEMENT FUNCTIONS
// ============================================================================

/**
 * CREATING A NEW 3D MODEL
 * ========================
 * 
 * This function dynamically allocates all structures necessary
 * for a 3D model. Dynamic allocation is crucial on Apple IIGS
 * because the stack is limited and cannot contain large arrays.
 * 
 * ALLOCATION STRATEGY:
 * 1. Main Model3D structure allocation
 * 2. Vertex array allocation (MAX_VERTICES elements)
 * 3. Face array allocation (MAX_FACES elements)
 * 4. On failure: cleanup of previous allocations
 * 
 * ERROR HANDLING:
 * - Check each allocation
 * - Automatic cascade cleanup on partial failure
 * - Return NULL if unable to allocate
 */
Model3D* createModel3D(void) {
    // Step 1: Main structure allocation
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    int n = MAX_VERTICES;
    model->vertices.vertex_count = n;
    
    // Step 2: Allocate vertex arrays using malloc (handles bank crossing better)
    // Note: malloc() should handle bank boundaries better than NewHandle()
    model->vertices.x = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.y = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.z = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.xo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.yo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.zo = (Fixed32*)malloc(n * sizeof(Fixed32));
    model->vertices.x2d = (int*)malloc(n * sizeof(int));
    model->vertices.y2d = (int*)malloc(n * sizeof(int));
    
    if (!model->vertices.x || !model->vertices.y || !model->vertices.z ||
        !model->vertices.xo || !model->vertices.yo || !model->vertices.zo ||
        !model->vertices.x2d || !model->vertices.y2d) {
        printf("Error: Unable to allocate memory for vertex arrays\n");
        keypress();
        // Allocation failed, cleanup
        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        free(model);
        return NULL;
    }
    
    // Set dummy handles to NULL (not used with malloc)
    model->vertices.xHandle = NULL;
    model->vertices.yHandle = NULL;
    model->vertices.zHandle = NULL;
    model->vertices.xoHandle = NULL;
    model->vertices.yoHandle = NULL;
    model->vertices.zoHandle = NULL;
    model->vertices.x2dHandle = NULL;
    model->vertices.y2dHandle = NULL;
    
    // Step 3: Face array allocation using parallel arrays (like vertices)
    // Each element stored separately to fit 32KB limit per allocation
    int nf = MAX_FACES;
    
    // Allocate vertex count array: nf * 4 bytes = 24KB
    model->faces.vertex_count = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_count) {
        printf("Error: Unable to allocate memory for face vertex_count array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model);
        return NULL;
    }
    
    // Allocate SINGLE packed buffer for all vertex indices
    // Estimate: average 3.5 indices per face (mix of triangles and quads)
    // For 6000 faces: ~21KB. We allocate conservatively at 5 per face = 120KB max
    int estimated_total_indices = nf * 5;
    model->faces.vertex_indices_buffer = (int*)malloc(estimated_total_indices * sizeof(int));
    if (!model->faces.vertex_indices_buffer) {
        printf("Error: Unable to allocate memory for vertex_indices_buffer\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model);
        return NULL;
    }
    
    // Allocate offset array: one offset per face into the packed buffer
    model->faces.vertex_indices_ptr = (int*)malloc(nf * sizeof(int));
    if (!model->faces.vertex_indices_ptr) {
        printf("Error: Unable to allocate memory for vertex_indices_ptr array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model);
        return NULL;
    }
    
    // Allocate z_max array: nf * 4 bytes = 24KB
    model->faces.z_max = (Fixed32*)malloc(nf * sizeof(Fixed32));
    if (!model->faces.z_max) {
        printf("Error: Unable to allocate memory for face z_max array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model);
        return NULL;
    }
    
    // Allocate display_flag array: nf * 4 bytes = 24KB
    model->faces.display_flag = (int*)malloc(nf * sizeof(int));
    if (!model->faces.display_flag) {
        printf("Error: Unable to allocate memory for face display_flag array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model);
        return NULL;
    }
    
    // Initialize structure
    model->faces.vertex_countHandle = NULL;
    model->faces.vertex_indicesBufferHandle = NULL;
    model->faces.vertex_indicesPtrHandle = NULL;
    model->faces.z_maxHandle = NULL;
    model->faces.display_flagHandle = NULL;
    model->faces.sorted_face_indicesHandle = NULL;
    model->faces.total_indices = 0;
    
    // Allocate sorted_face_indices array: nf * 4 bytes = 24KB max
    model->faces.sorted_face_indices = (int*)malloc(nf * sizeof(int));
    if (!model->faces.sorted_face_indices) {
        printf("Error: Unable to allocate memory for sorted_face_indices array\n");
        keypress();
        free(model->vertices.x);
        free(model->vertices.y);
        free(model->vertices.z);
        free(model->vertices.xo);
        free(model->vertices.yo);
        free(model->vertices.zo);
        free(model->vertices.x2d);
        free(model->vertices.y2d);
        free(model->faces.vertex_count);
        free(model->faces.vertex_indices_buffer);
        free(model->faces.vertex_indices_ptr);
        free(model->faces.z_max);
        free(model->faces.display_flag);
        free(model);
        return NULL;
    }
    
    return model;
}

/**
 * DESTROYING A 3D MODEL
 * ======================
 * 
 * This function properly frees all memory allocated for a
 * 3D model. It follows the LIFO principle (Last In, First Out) for
 * cleanup: last allocated = first freed.
 * 
 * CLEANUP ORDER:
 * 1. Face array (if allocated)
 * 2. Vertex array (if allocated)  
 * 3. Main structure
 * 
 * SAFETY:
 * - NULL check to avoid segmentation errors
 * - Selective cleanup based on successful allocations
 */
void destroyModel3D(Model3D* model) {
    if (model != NULL) {
        // Free all vertex arrays
        if (model->vertices.x) free(model->vertices.x);
        if (model->vertices.y) free(model->vertices.y);
        if (model->vertices.z) free(model->vertices.z);
        if (model->vertices.xo) free(model->vertices.xo);
        if (model->vertices.yo) free(model->vertices.yo);
        if (model->vertices.zo) free(model->vertices.zo);
        if (model->vertices.x2d) free(model->vertices.x2d);
        if (model->vertices.y2d) free(model->vertices.y2d);
        
        // Free all face arrays (now simplified with packed buffer)
        if (model->faces.vertex_count) free(model->faces.vertex_count);
        if (model->faces.vertex_indices_buffer) free(model->faces.vertex_indices_buffer);
        if (model->faces.vertex_indices_ptr) free(model->faces.vertex_indices_ptr);
        if (model->faces.z_max) free(model->faces.z_max);
        if (model->faces.display_flag) free(model->faces.display_flag);
        if (model->faces.sorted_face_indices) free(model->faces.sorted_face_indices);
        
        // Free main structure
        free(model);
    }
}

/**
 * COMPLETE 3D MODEL LOADING
 * ==========================
 * 
 * This function coordinates the complete loading of an OBJ file
 * by successively calling the vertex and face reading functions.
 * 
 * LOADING PIPELINE:
 * 1. Input parameter validation
 * 2. Read vertices from file
 * 3. Read faces from file  
 * 4. Update counters in structure
 * 
 * ERROR HANDLING:
 * - Vertex reading failure: immediate stop
 * - Face reading failure: warning but continue
 *   (vertices-only model remains usable)
 */
int loadModel3D(Model3D* model, const char* filename) {
    // Input parameter validation
    if (model == NULL || filename == NULL) {
        return -1;  // Invalid parameters
    }
    
    // Step 1: Read vertices from OBJ file
    // --- MAJ du compteur global pour la vérification des indices de faces ---
    readVertices_last_count = model->vertices.vertex_count;
    
    int vcount = readVertices(filename, &model->vertices, MAX_VERTICES);
    if (vcount < 0) {
        return -1;  // Critical failure: unable to read vertices
    }
    model->vertices.vertex_count = vcount;
    
    // Step 2: Read faces from OBJ file (using chunked allocation)
    // This function handles reading faces into 2 chunks transparently
    int fcount = readFaces_model(filename, model);
    if (fcount < 0) {
        // Critical failure: unable to read faces
        printf("\nWarning: Unable to read faces\n");
        model->faces.face_count = 0;  // No faces available
    } else {
        model->faces.face_count = fcount;
    }
    
    return 0;  // Success: model loaded (with or without faces)
}

// ============================================================================
//                    USER INTERFACE FUNCTIONS
// ============================================================================

/**
 * OBSERVER PARAMETER INPUT
 * ========================
 * 
 * This function presents a text interface to allow the
 * user to specify 3D visualization parameters.
 * 
 * REQUESTED PARAMETERS:
 * - Horizontal angle: rotation around Y-axis (left/right view)
 * - Vertical angle: rotation around X-axis (up/down view)
 * - Distance: observer distance (zoom)
 * - Screen rotation angle: final rotation in 2D plane
 * 
 * ERROR HANDLING:
 * - Default values if input failure
 * - Automatic string->Fixed32 conversion with atof() then FLOAT_TO_FIXED
 */
void getObserverParams(ObserverParams* params) {
    char input[50];  // Buffer for user input
    
    // Display section header
    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values)\n");
    printf("(Enter 'debug' to see values used)\n");
    
    // Input horizontal angle (rotation around Y)
    printf("Horizontal angle (degrees, default 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_h = FLOAT_TO_FIXED(30.0);     // Default value if ENTER (Fixed Point)
        } else {
            params->angle_h = FLOAT_TO_FIXED(atof(input));  // String->Fixed32 conversion
        }
    } else {
        params->angle_h = FLOAT_TO_FIXED(30.0);         // Default value if error (Fixed Point)
    }
    
    // Input vertical angle (rotation around X)  
    printf("Vertical angle (degrees, default 20): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_v = FLOAT_TO_FIXED(20.0);     // Default value if ENTER (Fixed Point)
        } else {
            params->angle_v = FLOAT_TO_FIXED(atof(input));  // String->Fixed32 conversion
        }
    } else {
        params->angle_v = FLOAT_TO_FIXED(20.0);         // Default value if error (Fixed Point)
    }
    

    // Input screen rotation angle (final 2D rotation)
    printf("Screen rotation angle (degrees, default 0): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_w = FLOAT_TO_FIXED(0.0);      // Default value if ENTER (Fixed Point)
        } else {
            params->angle_w = FLOAT_TO_FIXED(atof(input));  // String->Fixed32 conversion
        }
    } else {
        params->angle_w = FLOAT_TO_FIXED(0.0);          // No rotation by default (Fixed Point)
    }

    // Input observation distance (zoom/perspective)
    printf("Distance (default 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->distance = FLOAT_TO_FIXED(30.0);    // Default value if ENTER (Fixed Point)
        } else {
            params->distance = FLOAT_TO_FIXED(atof(input)); // String->Fixed32 conversion
        }
    } else {
        params->distance = FLOAT_TO_FIXED(30.0);        // Default distance: balanced view (Fixed Point)
    }
}

/**
 * MODEL INFORMATION DISPLAY
 * =========================
 * 
 * Displays a statistical summary of the loaded 3D model:
 * number of vertices and number of faces.
 * 
 * This function serves as quick validation that loading
 * completed correctly.
 */
void displayModelInfo(Model3D* model) {
    // printf("\nAnalysis summary:\n");
    // printf("====================\n");
    // printf("Number of vertices (3D points) found: %d\n", model->vertices.vertex_count);
    // printf("Number of faces found: %d\n", model->faces.face_count);
}

/**
 * DETAILED RESULTS DISPLAY
 * ========================
 * 
 * This function displays all vertex coordinates through
 * the different stages of the rendering pipeline:
 * 1. Original coordinates (x, y, z)
 * 2. Transformed coordinates (xo, yo, zo)
 * 3. 2D projected coordinates (x2d, y2d)
 * 
 * Also displays the list of faces with their vertices.
 * Useful for debugging and calculation verification.
 */
void displayResults(Model3D* model) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    // printf("\nComplete coordinates (Original -> 3D -> 2D):\n");
    // printf("-----------------------------------------------\n");
    // for (i = 0; i < vtx->vertex_count; i++) {
    //     if (vtx->x2d[i] >= 0 && vtx->y2d[i] >= 0) {
    //         printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (%d,%d)\n",
    //                i + 1,
    //                FIXED_TO_FLOAT(vtx->x[i]), FIXED_TO_FLOAT(vtx->y[i]), FIXED_TO_FLOAT(vtx->z[i]),
    //                FIXED_TO_FLOAT(vtx->xo[i]), FIXED_TO_FLOAT(vtx->yo[i]), FIXED_TO_FLOAT(vtx->zo[i]),
    //                vtx->x2d[i], vtx->y2d[i]);
    //     } else {
    //         printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (invisible)\n",
    //                i + 1,
    //                FIXED_TO_FLOAT(vtx->x[i]), FIXED_TO_FLOAT(vtx->y[i]), FIXED_TO_FLOAT(vtx->z[i]),
    //                FIXED_TO_FLOAT(vtx->xo[i]), FIXED_TO_FLOAT(vtx->yo[i]), FIXED_TO_FLOAT(vtx->zo[i]));
    //     }
    // }
    
    if (model->faces.face_count > 0) {
        // printf("\nFace list:\n");
        // printf("----------------\n");
        for (i = 0; i < model->faces.face_count; i++) {
            // printf("  Face %3d (%d vertices, z_max=%.2f): ", i + 1, model->faces.vertex_count[i], FIXED_TO_FLOAT(model->faces.z_max[i]));
            int offset = model->faces.vertex_indices_ptr[i];
            // for (j = 0; j < model->faces.vertex_count[i]; j++) {
            //     printf("%d", model->faces.vertex_indices_buffer[offset + j]);
            //     if (j < model->faces.vertex_count[i] - 1) printf("-");
            // }
            // printf("\n");
            // printf("       Coordinates of vertices of this face:\n");
            for (j = 0; j < model->faces.vertex_count[i]; j++) {
                int vertex_idx = model->faces.vertex_indices_buffer[offset + j] - 1;
                if (vertex_idx >= 0 && vertex_idx < vtx->vertex_count) {
                    // printf("         Vertex %d: (%.2f,%.2f,%.2f) -> (%d,%d)\n",
                    //     model->faces.vertex_indices_buffer[offset + j],
                    //     FIXED_TO_FLOAT(vtx->x[vertex_idx]), FIXED_TO_FLOAT(vtx->y[vertex_idx]), FIXED_TO_FLOAT(vtx->z[vertex_idx]),
                    //     vtx->x2d[vertex_idx], vtx->y2d[vertex_idx]);
                } else {
                    // printf("         Vertex %d: ERROR - Index out of bounds!\n", model->faces.vertex_indices_buffer[offset + j]);
                }
            }
            // printf("\n");
        }
        drawPolygons(model, model->faces.vertex_count, model->faces.face_count, vtx->vertex_count);
    }
}

/**
 * ULTRA-FAST FUNCTION: Combined Transformation + Projection
 * ==========================================================
 */
void processModelFast(Model3D* model, ObserverParams* params, const char* filename) {
    int i;
    Fixed32 rad_h, rad_v, rad_w;
    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo, x2d_temp, y2d_temp;
    
    // Direct table access - ultra-fast! (no function calls)
    rad_h = deg_to_rad_table[FIXED_TO_INT(params->angle_h)];
    rad_v = deg_to_rad_table[FIXED_TO_INT(params->angle_v)];
    rad_w = deg_to_rad_table[FIXED_TO_INT(params->angle_w)];
    
    // Pre-calculate ALL trigonometric values in Fixed32 (ultra-fast)
    cos_h = cos_fixed(rad_h);
    sin_h = sin_fixed(rad_h);
    cos_v = cos_fixed(rad_v);
    sin_v = sin_fixed(rad_v);
    cos_w = cos_fixed(rad_w);
    sin_w = sin_fixed(rad_w);
    
    // Pre-calculate all trigonometric products in Fixed32 - using 64-bit multiply
    const Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
    const Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
    const Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
    const Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);
    const Fixed32 scale = FLOAT_TO_FIXED(100.0);
    const Fixed32 centre_x_f = FLOAT_TO_FIXED((float)CENTRE_X);
    const Fixed32 centre_y_f = FLOAT_TO_FIXED((float)CENTRE_Y);
    const Fixed32 distance = params->distance;
    
    // Performance measurement
    long start_transform_ticks = GetTick();
    
    // 100% Fixed32 loop - ZERO conversions, maximum speed!
    VertexArrays3D* vtx = &model->vertices;
    
    for (i = 0; i < vtx->vertex_count; i++) {
        x = vtx->x[i];
        y = vtx->y[i];
        z = vtx->z[i];
        // 3D transformation in pure Fixed32 (64-bit multiply)
        Fixed32 term1 = FIXED_MUL_64(x, cos_h_cos_v);
        Fixed32 term2 = FIXED_MUL_64(y, sin_h_cos_v);
        Fixed32 term3 = FIXED_MUL_64(z, sin_v);
        zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), distance);
        if (zo > 0) {
            xo = FIXED_ADD(FIXED_NEG(FIXED_MUL_64(x, sin_h)), FIXED_MUL_64(y, cos_h));
            yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(FIXED_MUL_64(x, cos_h_sin_v)), FIXED_MUL_64(y, sin_h_sin_v)), FIXED_MUL_64(z, cos_v));
            vtx->zo[i] = zo;
            vtx->xo[i] = xo;
            vtx->yo[i] = yo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_temp = FIXED_ADD(FIXED_MUL_64(xo, inv_zo), centre_x_f);
            y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo, inv_zo));
            vtx->x2d[i] = FIXED_TO_INT(FIXED_ADD(FIXED_SUB(FIXED_MUL_64(cos_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(sin_w, FIXED_SUB(centre_y_f, y2d_temp))), centre_x_f));
            vtx->y2d[i] = FIXED_TO_INT(FIXED_SUB(centre_y_f, FIXED_ADD(FIXED_MUL_64(sin_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(cos_w, FIXED_SUB(centre_y_f, y2d_temp)))));
        } else {
            vtx->zo[i] = zo;
            vtx->xo[i] = 0;
            vtx->yo[i] = 0;
            vtx->x2d[i] = -1;
            vtx->y2d[i] = -1;
        }
    }
    
    long end_transform_ticks = GetTick();
    
    // Face sorting after transformation
    long start_calc_ticks = GetTick();
    calculateFaceDepths(model, NULL, model->faces.face_count);
    long end_calc_ticks = GetTick();
    
    // CRITICAL: Reset sorted_face_indices before each sort to prevent corruption
    for (i = 0; i < model->faces.face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }
    
    long start_sort_ticks = GetTick();
    sortFacesByDepth(model, model->faces.face_count);
    long end_sort_ticks = GetTick();
    
#if !PERFORMANCE_MODE
    printf("Transform+Project: %ld ticks (%.2f ms)\n", 
           end_transform_ticks - start_transform_ticks, 
           (end_transform_ticks - start_transform_ticks) * 1000.0 / 60.0);
    printf("calculateFaceDepths: %ld ticks (%.2f ms)\n", 
           end_calc_ticks - start_calc_ticks, 
           (end_calc_ticks - start_calc_ticks) * 1000.0 / 60.0);
    printf("sortFacesByDepth: %ld ticks (%.2f ms)\n", 
           end_sort_ticks - start_sort_ticks,
           (end_sort_ticks - start_sort_ticks) * 1000.0 / 60.0);
    printf("\nHit a key to continue...\n");
    keypress();
#endif
}

// ============================================================================
//                    BASIC FUNCTION IMPLEMENTATIONS
// ============================================================================

/**
 * VERTEX READING FROM OBJ FILE
 * ============================
 * 
 * This function parses an OBJ format file to extract
 * vertices (3D points). It searches for lines starting with "v "
 * and extracts X, Y, Z coordinates.
 * 
 * OBJ FORMAT FOR VERTICES:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14159
 * 
 * ALGORITHM:
 * 1. Open file in read mode
 * 2. Read line by line with fgets()
 * 3. Detect "v " lines with character verification
 * 4. Extract coordinates with sscanf()
 * 5. Store in array with bounds checking
 * 6. Progressive display for user feedback
 * 
 * ERROR HANDLING:
 * - File opening verification
 * - Array overflow protection
 * - Coordinate format validation
 */
int readVertices(const char* filename, VertexArrays3D* vtx, int max_vertices) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("[DEBUG] readVertices: fopen failed\n");
        printf("Error: Unable to open file '%s'\n", filename);
        printf("Check that the file exists and you have read permissions.\n");
        return -1;  // Return -1 on error
    }
    
    printf("\nReading vertices from file...'%s':\n", filename);
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // printf("%3d: %s", line_number, line);
        if (line[0] == 'v' && line[1] == ' ') {
            //printf("[DEBUG] readVertices: found vertex line %d\n", line_number);
            if (vertex_count < max_vertices) {
                float x, y, z;  // Temporary reading in float
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vtx->x[vertex_count] = FLOAT_TO_FIXED(x);
                    vtx->y[vertex_count] = FLOAT_TO_FIXED(y);
                    vtx->z[vertex_count] = FLOAT_TO_FIXED(z);
                    vertex_count++;
                    if (vertex_count % 10 == 0) printf("..");

                } else {
                    printf("[\nDEBUG] readVertices: sscanf failed at line %d: %s\n", line_number, line);
                    keypress();
                }
            } else {
                printf("\n[DEBUG] readVertices: vertex limit reached (%d)\n", max_vertices);
                keypress();
            }
        }
        line_number++;
    }
    printf("\n");
    printf("Reading vertices finished : %d vertices read.\n", vertex_count);

    // Close file
    fclose(file);
    
    // printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
    return vertex_count;  // Return the number of vertices read
}

// Function to read faces into parallel arrays in FaceArrays3D structure
int readFaces_model(const char* filename, Model3D* model) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int face_count = 0;
    int i;
    
    // Validate model structure
    if (model == NULL || model->faces.vertex_count == NULL) {
        printf("Error: Invalid model structure for readFaces_model\n");
        return -1;
    }
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;
    }
    
    printf("\nReading faces from file '%s' :\n", filename);
    
    int buffer_pos = 0;  // Current position in the packed buffer
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // Check if line starts with "f " (face)
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < MAX_FACES) {
                // Initialize face data
                model->faces.vertex_count[face_count] = 0;
                model->faces.display_flag[face_count] = 1;  // Displayable by default
                model->faces.vertex_indices_ptr[face_count] = buffer_pos;  // Store offset to this face's indices
                
                // Parse vertices from this face
                char *ptr = line + 2;  // Start after "f "
                int temp_indices[MAX_FACE_VERTICES];
                int temp_vertex_count = 0;
                int invalid_index_found = 0;
                
                // Parse character by character
                while (*ptr != '\0' && *ptr != '\n' && temp_vertex_count < MAX_FACE_VERTICES) {
                    // Skip spaces and tabs
                    while (*ptr == ' ' || *ptr == '\t') ptr++;
                    
                    if (*ptr == '\0' || *ptr == '\n') break;
                    
                    // Read the number
                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }
                    
                    // Skip texture/normal data (after /)
                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }
                    
                    // Validate vertex index
                    if (vertex_index >= 1) {
                        if (vertex_index > readVertices_last_count) {
                            // Index out of bounds
                            invalid_index_found = 1;
                        }
                        temp_indices[temp_vertex_count] = vertex_index;
                        temp_vertex_count++;
                    }
                }
                
                // Check for errors
                if (invalid_index_found) {
                    printf("\nERROR: Face at line %d references vertex index > %d vertices\n", 
                           line_number, readVertices_last_count);
                    fclose(file);
                    return -1;
                } else {
                    // Store valid indices into the packed buffer
                    for (i = 0; i < temp_vertex_count; i++) {
                        model->faces.vertex_indices_buffer[buffer_pos++] = temp_indices[i];
                    }
                    model->faces.vertex_count[face_count] = temp_vertex_count;
                    model->faces.total_indices += temp_vertex_count;
                    
                    // Skip faces with zero vertices
                    if (model->faces.vertex_count[face_count] > 0) {
                        face_count++;
                        if (face_count % 10 == 0) {printf(".");}
                    } else {
                        printf("     -> WARNING: Face without valid vertices ignored\n");
                    }
                }
            } else {
                printf("     -> WARNING: Face limit reached (%d)\n", MAX_FACES);
            }
        }
        
        line_number++;
    }
    
    // Close file
    fclose(file);
    
    model->faces.face_count = face_count;
    
    // Initialize sorted_face_indices with identity mapping (will be sorted later)
    for (i = 0; i < face_count; i++) {
        model->faces.sorted_face_indices[i] = i;
    }
    
    printf("\nReading faces finished : %d faces read.\n", face_count);
    return face_count;
}
void transformToObserver(VertexArrays3D* vtx, Fixed32 angle_h, Fixed32 angle_v, Fixed32 distance) {
    int i;
    Fixed32 rad_h, rad_v;
    Fixed32 cos_h, sin_h, cos_v, sin_v;
    Fixed32 x, y, z;
    rad_h = FIXED_MUL_64(angle_h, FIXED_PI_180);
    rad_v = FIXED_MUL_64(angle_v, FIXED_PI_180);
    cos_h = cos_fixed(rad_h);
    sin_h = sin_fixed(rad_h);
    cos_v = cos_fixed(rad_v);
    sin_v = sin_fixed(rad_v);
    Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
    Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
    Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
    Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);
#if !PERFORMANCE_MODE
    printf("\nTransformation to observer system (Fixed Point):\n");
    printf("Horizontal angle: %.1f degrees\n", FIXED_TO_FLOAT(angle_h));
    printf("Vertical angle: %.1f degrees\n", FIXED_TO_FLOAT(angle_v));
    printf("Distance: %.3f\n", FIXED_TO_FLOAT(distance));
    printf("==========================================\n");
#endif
    for (i = 0; i < vtx->vertex_count; i++) {
        x = vtx->x[i];
        y = vtx->y[i];
        z = vtx->z[i];
        vtx->xo[i] = FIXED_ADD(
            FIXED_MUL_64(x, cos_h_cos_v),
            FIXED_MUL_64(y, sin_h_cos_v)
        );
        vtx->yo[i] = FIXED_ADD(
            FIXED_SUB(
                FIXED_MUL_64(-x, cos_h_sin_v),
                FIXED_MUL_64(y, sin_h_sin_v)
            ),
            FIXED_MUL_64(z, cos_v)
        );
        vtx->zo[i] = FIXED_ADD(
            FIXED_ADD(
                FIXED_MUL_64(-x, sin_h),
                FIXED_MUL_64(-y, cos_h)
            ),
            distance
        );
    }
}

// Function to project 3D coordinates onto 2D screen - FIXED POINT VERSION
void projectTo2D(VertexArrays3D* vtx, Fixed32 angle_w) {
    int i;
    Fixed32 rad_w;
    Fixed32 cos_w, sin_w;
    Fixed32 x2d_temp, y2d_temp;
    rad_w = FIXED_MUL_64(angle_w, FIXED_PI_180);
    cos_w = cos_fixed(rad_w);
    sin_w = sin_fixed(rad_w);
    const Fixed32 scale = INT_TO_FIXED(100);
    const Fixed32 centre_x_f = INT_TO_FIXED(CENTRE_X);
    const Fixed32 centre_y_f = INT_TO_FIXED(CENTRE_Y);
#if !PERFORMANCE_MODE
    printf("\nProjection on 2D screen (Fixed Point):\n");
    printf("Rotation angle: %.1f degrees\n", FIXED_TO_FLOAT(angle_w));
    printf("Screen center: (%d, %d)\n", CENTRE_X, CENTRE_Y);
    printf("===========================\n");
#endif
    for (i = 0; i < vtx->vertex_count; i++) {
        if (vtx->zo[i] > 0) {
            Fixed32 xo = vtx->xo[i];
            Fixed32 yo = vtx->yo[i];
            Fixed32 inv_zo = FIXED_DIV_64(scale, vtx->zo[i]);
            x2d_temp = FIXED_ADD(FIXED_MUL_64(xo, inv_zo), centre_x_f);
            y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo, inv_zo));
            vtx->x2d[i] = FIXED_TO_INT(FIXED_ADD(FIXED_SUB(FIXED_MUL_64(cos_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(sin_w, FIXED_SUB(centre_y_f, y2d_temp))), centre_x_f));
            vtx->y2d[i] = FIXED_TO_INT(FIXED_SUB(centre_y_f, FIXED_ADD(FIXED_MUL_64(sin_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(cos_w, FIXED_SUB(centre_y_f, y2d_temp)))));
        } else {
            vtx->x2d[i] = -1;
            vtx->y2d[i] = -1;
        }
    }
}

/**
 * CALCULATING MINIMUM FACE DEPTHS AND VISIBILITY FLAGS
 * =====================================================
 * 
 * This function calculates for each face:
 * 1. The minimum depth (z_min) of all its vertices in the observer coordinate system
 * 2. The display visibility flag based on vertex positions relative to camera
 * 
 * The z_min value is used for face sorting during rendering (painter's algorithm).
 * We use minimum (closest point) for correct occlusion in the painter's algorithm.
 * The display_flag is used to cull faces that have vertices behind the camera.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with coordinates in observer system
 *   faces      : Array of faces to process  
 *   face_count : Number of faces
 * 
 * ALGORITHM:
 *   For each face:
 *   - Initialize z_min with very large value (9999.0)
 *   - Initialize display_flag as true (displayable)
 *   - For each vertex of the face:
 *     * Check if vertex is behind camera (zo <= 0)
 *     * If ANY vertex is behind camera, set display_flag = false
 *     * Update z_min with minimum zo value found (closest vertex)
 *   - Store both z_min and display_flag in the face structure
 * 
 * CULLING LOGIC:
 *   - If ANY vertex has zo <= 0, the entire face is marked as non-displayable
 *   - This prevents rendering artifacts from perspective projection errors
 *   - Improves performance by eliminating faces early in the pipeline
 * 
 * NOTES:
 *   - Must be called AFTER transformToObserver() or processModelFast()
 *   - Uses zo coordinates (observer system depth)
 *   - Lower z_min value means face is closer to camera (should draw first in painter's algorithm)
 *   - display_flag = 1 means visible, 0 means hidden (behind camera)
 */
void calculateFaceDepths(Model3D* model, Face3D* faces, int face_count) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* face_arrays = &model->faces;
    
    for (i = 0; i < face_count; i++) {
        Fixed32 z_min = FLOAT_TO_FIXED(9999.0);  // Initialize to very large value
        int display_flag = 1;
        
        // Access indices from the packed buffer using the offset
        int offset = face_arrays->vertex_indices_ptr[i];
        for (j = 0; j < face_arrays->vertex_count[i]; j++) {
            int vertex_idx = face_arrays->vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx >= 0) {
                if (vtx->zo[vertex_idx] <= 0) display_flag = 0;
                if (vtx->zo[vertex_idx] < z_min) z_min = vtx->zo[vertex_idx];  // Find minimum (closest)
            }
        }
        face_arrays->z_max[i] = z_min;  // Store minimum depth for sorting
        face_arrays->display_flag[i] = display_flag;
    }
}

/**
 * FACE SORTING BY DEPTH (OPTIMIZED VERSION)
 * ==========================================
 * 
 * This function sorts faces in descending order of z_max depth
 * to implement the painter's algorithm. The farthest faces
 * (highest z_max) are placed first in the array to be
 * drawn first, and the nearest ones last.
 * 
 * PARAMETERS:
 *   faces      : Array of faces to sort
 *   face_count : Number of faces
 * 
 * ALGORITHMS:
 *   - Insertion sort for small collections (< 10 faces) - O(n²) but fast
 *   - Quick sort (quicksort) for large collections - O(n log n) on average
 *   - Already sorted array detection to avoid unnecessary sorting - O(n)
 * 
 * OPTIMIZATIONS:
 *   - Adaptive threshold based on size
 *   - Pre-sort verification for already ordered arrays
 *   - Median-of-three pivoting for stable quicksort
 * 
 * NOTES:
 *   - Must be called AFTER calculateFaceDepths()
 *   - Sort order: descending z_max (farthest to nearest)
 */
void sortFacesByDepth(Model3D* model, int face_count) {
    // Sort faces by z_max in descending order (farthest to nearest)
    // Optimized: insertion sort for small arrays, quicksort for large
    
    if (face_count <= 1) {
        return;
    }
    
    FaceArrays3D* faces = &model->faces;
    
    // For small arrays (<=16), use insertion sort (faster due to low overhead)
    if (face_count <= 16) {
        sortFacesByDepth_insertion(faces, face_count);
    } else {
        // For larger arrays, use quicksort
        sortFacesByDepth_quicksort(faces, 0, face_count - 1);
    }
}

// Helper macro to swap face indices in the sorted_face_indices array
// (We swap indices, not the faces themselves, to keep the buffer intact)
#define SWAP_FACE(faces, i, j) \
    do { \
        int temp_idx = faces->sorted_face_indices[i]; \
        faces->sorted_face_indices[i] = faces->sorted_face_indices[j]; \
        faces->sorted_face_indices[j] = temp_idx; \
    } while (0)

/**
 * Insertion sort for small arrays (optimized for sorted_face_indices)
 * Now we sort indices by comparing their depth values (z_max)
 */
void sortFacesByDepth_insertion(FaceArrays3D* faces, int face_count) {
    int i, j;
    
    for (i = 1; i < face_count; i++) {
        int face_i = faces->sorted_face_indices[i];
        int face_prev = faces->sorted_face_indices[i - 1];
        
        // If already in order (descending), skip
        if (faces->z_max[face_i] <= faces->z_max[face_prev]) {
            continue;
        }
        
        // Find insertion position
        j = i - 1;
        while (j >= 0) {
            int face_j = faces->sorted_face_indices[j];
            if (faces->z_max[face_j] >= faces->z_max[face_i]) break;
            // Shift index right
            SWAP_FACE(faces, j + 1, j);
            j--;
        }
        
        // Insert at position j+1
        faces->sorted_face_indices[j + 1] = face_i;
    }
}

/**
 * Quicksort for large arrays
 */
void sortFacesByDepth_quicksort(FaceArrays3D* faces, int low, int high) {
    if (low < high) {
        // For small partitions, use insertion sort
        if (high - low <= 16) {
            sortFacesByDepth_insertion_range(faces, low, high);
            return;
        }
        
        // Partition and recursively sort
        int pivot_index = sortFacesByDepth_partition(faces, low, high);
        sortFacesByDepth_quicksort(faces, low, pivot_index - 1);
        sortFacesByDepth_quicksort(faces, pivot_index + 1, high);
    }
}

/**
 * Insertion sort for a range (used by quicksort)
 */
void sortFacesByDepth_insertion_range(FaceArrays3D* faces, int low, int high) {
    int i, j;
    
    for (i = low + 1; i <= high; i++) {
        int face_i = faces->sorted_face_indices[i];
        int face_prev = faces->sorted_face_indices[i - 1];
        
        if (faces->z_max[face_i] <= faces->z_max[face_prev]) {
            continue;
        }
        
        j = i - 1;
        while (j >= low) {
            int face_j = faces->sorted_face_indices[j];
            if (faces->z_max[face_j] >= faces->z_max[face_i]) break;
            // Shift index right
            SWAP_FACE(faces, j + 1, j);
            j--;
        }
        
        // Insert at position j+1
        faces->sorted_face_indices[j + 1] = face_i;
    }
}

/**
 * Partition function for quicksort (median-of-three pivot)
 */
int sortFacesByDepth_partition(FaceArrays3D* faces, int low, int high) {
    // Median-of-three for better pivot selection
    int mid = low + (high - low) / 2;
    
    int face_low = faces->sorted_face_indices[low];
    int face_mid = faces->sorted_face_indices[mid];
    int face_high = faces->sorted_face_indices[high];
    
    if (faces->z_max[face_low] < faces->z_max[face_mid]) {
        SWAP_FACE(faces, low, mid);
    }
    if (faces->z_max[faces->sorted_face_indices[low]] < faces->z_max[face_high]) {
        SWAP_FACE(faces, low, high);
    }
    if (faces->z_max[faces->sorted_face_indices[mid]] < faces->z_max[faces->sorted_face_indices[high]]) {
        SWAP_FACE(faces, mid, high);
    }
    
    // Use median (now at low) as pivot
    Fixed32 pivot = faces->z_max[faces->sorted_face_indices[low]];
    SWAP_FACE(faces, low, high);  // Move pivot to end
    
    int i = low - 1;
    for (int j = low; j < high; j++) {
        if (faces->z_max[faces->sorted_face_indices[j]] >= pivot) {  // Descending order
            i++;
            SWAP_FACE(faces, i, j);
        }
    }
    
    SWAP_FACE(faces, i + 1, high);  // Move pivot to final position
    return i + 1;
}

// Function to draw polygons with QuickDraw
void drawPolygons(Model3D* model, int* vertex_count, int face_count, int vertex_count_total) {
    int i, j;
    VertexArrays3D* vtx = &model->vertices;
    FaceArrays3D* faces = &model->faces;
    Handle polyHandle;
    DynamicPolygon *poly;
    int min_x, max_x, min_y, max_y;
    int valid_faces_drawn = 0;
    int invalid_faces_skipped = 0;
    int triangle_count = 0;
    int quad_count = 0;
    Pattern pat;
    
    // Use global persistent handle to avoid repeated NewHandle/DisposeHandle
    // Each call allocates fresh if needed, but reuses same handle block
    if (globalPolyHandle == NULL) {
        int max_polySize = 2 + 8 + (4 * 4);  // Max for quad (4 vertices)
        globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
        if (globalPolyHandle == NULL) {
            printf("Error: Unable to allocate global polygon handle\n");
            return;
        }
    }
    
    polyHandle = globalPolyHandle;
    
    // Make sure handle is unlocked before locking
    if (poly_handle_locked) {
        HUnlock(polyHandle);
        poly_handle_locked = 0;
    }
    HLock(polyHandle);
    poly_handle_locked = 1;

    SetPenMode(0);
    // printf("\nDrawing polygons on screen:\n");
    FILE *face_log = NULL;  // Disabled face.log to save time
    // FILE *face_log = fopen("face.log", "w"); // Log file for face coordinates
    // if (!face_log) {
    //     printf("Erreur : impossible de créer ou ouvrir face.log\n");
    //     keypress();
    // }
    
    // Use sorted_face_indices to draw in correct depth order
    // Draw ALL faces - painter's algorithm handles occlusion
    int start_face = 0;
    int max_faces_to_draw = face_count;
    
    for (i = start_face; i < start_face + max_faces_to_draw; i++) {
        int face_id = faces->sorted_face_indices[i];
        if (faces->display_flag[face_id] == 0) continue;
        if (faces->vertex_count[face_id] >= 3) {
            int offset = faces->vertex_indices_ptr[face_id];
            if (face_log) {
                fprintf(face_log, "Face %d:\n", face_id);
                for (j = 0; j < faces->vertex_count[face_id]; j++) {
                    int vertex_idx = faces->vertex_indices_buffer[offset + j] - 1;
                    // Check for valid vertex index
                    if (vertex_idx >= 0 && vertex_idx < vtx->vertex_count) {
                        fprintf(face_log, "  Vertex %d: x2d=%d y2d=%d xo=%.4f yo=%.4f zo=%.4f\n",
                            vertex_idx,
                            vtx->x2d[vertex_idx],
                            vtx->y2d[vertex_idx],
                            FIXED_TO_FLOAT(vtx->xo[vertex_idx]),
                            FIXED_TO_FLOAT(vtx->yo[vertex_idx]),
                            FIXED_TO_FLOAT(vtx->zo[vertex_idx]));
                    } else {
                        fprintf(face_log, "  Vertex (invalid index): %d\n", faces->vertex_indices_buffer[offset + j]);
                    }
                }
            }
            // Calculate polySize for this specific face
            int polySize = 2 + 8 + (faces->vertex_count[face_id] * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;
            min_x = max_x = min_y = max_y = -1;
            for (j = 0; j < faces->vertex_count[face_id]; j++) {
                int vertex_idx = faces->vertex_indices_buffer[offset + j] - 1;
                // Only draw valid vertices
                if (vertex_idx >= 0 && vertex_idx < vtx->vertex_count) {
                    poly->polyPoints[j].h = mode / 320 * vtx->x2d[vertex_idx];
                    poly->polyPoints[j].v = vtx->y2d[vertex_idx];
                    if (min_x == -1 || vtx->x2d[vertex_idx] < min_x) min_x = vtx->x2d[vertex_idx];
                    if (max_x == -1 || vtx->x2d[vertex_idx] > max_x) max_x = vtx->x2d[vertex_idx];
                    if (min_y == -1 || vtx->y2d[vertex_idx] < min_y) min_y = vtx->y2d[vertex_idx];
                    if (max_y == -1 || vtx->y2d[vertex_idx] > max_y) max_y = vtx->y2d[vertex_idx];
                }
            }
            poly->polyBBox.h1 = min_x;
            poly->polyBBox.v1 = min_y;
            poly->polyBBox.h2 = max_x;
            poly->polyBBox.v2 = max_y;
            SetSolidPenPat(14);
            GetPenPat(pat);
            FillPoly(polyHandle, pat);
            SetSolidPenPat(7);
            FramePoly(polyHandle);
            valid_faces_drawn++;
        } else {
            invalid_faces_skipped++;
        }
    }
    if (face_log) fclose(face_log);
    
    // Cleanup: unlock handle but keep it allocated for next frame
    if (poly_handle_locked) {
        HUnlock(polyHandle);
        poly_handle_locked = 0;
    }
}
/**
 * DEBUG DATA SAVE
 * ===============
 * 
 * This function saves all 3D model data to a text file
 * to allow analysis of display problems.
 * 
 * DEBUG FILE CONTENT:
 * - Complete vertex list with 3D and 2D coordinates
 * - Complete face list with indices and calculated coordinates
 * - Model statistics
 */
void saveDebugData(Model3D* model, const char* debug_filename) {
    FILE *debug_file;
    int i, j;
    int triangle_count = 0, quad_count = 0, other_count = 0;
    VertexArrays3D* vtx = &model->vertices;
    // Open debug file for writing
    debug_file = fopen(debug_filename, "w");
    if (debug_file == NULL) {
        printf("Error: Unable to create debug file '%s'\n", debug_filename);
        return;
    }
    // Face statistics
    for (i = 0; i < model->faces.face_count; i++) {
        if (model->faces.vertex_count[i] == 3) triangle_count++;
        else if (model->faces.vertex_count[i] == 4) quad_count++;
        else other_count++;
    }
    fprintf(debug_file, "Triangles detected: %d\n", triangle_count);
    fprintf(debug_file, "Quadrilaterals detected: %d\n", quad_count);
    fprintf(debug_file, "Other polygons: %d\n", other_count);
    fprintf(debug_file, "\n");
    // Complete vertex list
    fprintf(debug_file, "=== VERTICES ===\n");
    fprintf(debug_file, "Format: Index | X3D Y3D Z3D | X2D Y2D\n");
    fprintf(debug_file, "--------------------------------------\n");
    for (i = 0; i < vtx->vertex_count; i++) {
        fprintf(debug_file, "V%03d | %8.3f %8.3f %8.3f | %4d %4d\n",
                i + 1,
                FIXED_TO_FLOAT(vtx->x[i]), FIXED_TO_FLOAT(vtx->y[i]), FIXED_TO_FLOAT(vtx->z[i]),
                vtx->x2d[i], vtx->y2d[i]);
    }
    fprintf(debug_file, "\n");
    // Complete face list
    fprintf(debug_file, "=== FACES ===\n");
    for (i = 0; i < model->faces.face_count; i++) {
        fprintf(debug_file, "Face F%03d (%d vertices):\n", i + 1, model->faces.vertex_count[i]);
        fprintf(debug_file, "  Indices: ");
        int offset = model->faces.vertex_indices_ptr[i];
        for (j = 0; j < model->faces.vertex_count[i]; j++) {
            fprintf(debug_file, "V%d", model->faces.vertex_indices_buffer[offset + j]);
            if (j < model->faces.vertex_count[i] - 1) fprintf(debug_file, ", ");
        }
        fprintf(debug_file, "\n");
        // 3D and 2D coordinates of each vertex of the face
        fprintf(debug_file, "  Coordinates:\n");
        for (j = 0; j < model->faces.vertex_count[i]; j++) {
            int vertex_idx = model->faces.vertex_indices_buffer[offset + j] - 1; // Convert base-1 to base-0
            if (vertex_idx >= 0 && vertex_idx < vtx->vertex_count) {
                fprintf(debug_file, "    V%d: 3D(%.3f, %.3f, %.3f) -> 2D(%d, %d)\n",
                        model->faces.vertex_indices_buffer[offset + j],
                        FIXED_TO_FLOAT(vtx->x[vertex_idx]), FIXED_TO_FLOAT(vtx->y[vertex_idx]), FIXED_TO_FLOAT(vtx->z[vertex_idx]),
                        vtx->x2d[vertex_idx], vtx->y2d[vertex_idx]);
            } else {
                fprintf(debug_file, "    V%d: ERROR - Index out of bounds!\n", model->faces.vertex_indices_buffer[offset + j]);
            }
        }
        fprintf(debug_file, "\n");
    }
    // Integrity check
    fprintf(debug_file, "=== INTEGRITY CHECK ===\n");
    int errors = 0;
    for (i = 0; i < model->faces.face_count; i++) {
        int offset = model->faces.vertex_indices_ptr[i];
        for (j = 0; j < model->faces.vertex_count[i]; j++) {
            int vertex_idx = model->faces.vertex_indices_buffer[offset + j] - 1;
            if (vertex_idx < 0 || vertex_idx >= vtx->vertex_count) {
                fprintf(debug_file, "ERROR: Face F%d references non-existent vertex V%d (index %d out of bounds [1-%d])\n",
                        i + 1, model->faces.vertex_indices_buffer[offset + j], vertex_idx + 1, vtx->vertex_count);
                errors++;
            }
        }
    }
    if (errors == 0) {
        fprintf(debug_file, "No errors detected - All indices are valid.\n");
    } else {
        fprintf(debug_file, "TOTAL: %d errors detected!\n", errors);
    }
    // Close file
    fclose(debug_file);
}


void DoColor() {
        Rect r;
        unsigned char pstr[4];  // Pascal string: [length][characters...]]

        SetRect (&r, 0, 1, mode / 320 *10, 11);
        for (int i = 0; i < 16; i++) {
            SetSolidPenPat(i);
            PaintRect(&r);

            if (i == 0) {
                SetSolidPenPat(15); // White frame for black background
                FrameRect(&r);
            }

            MoveTo(r.h1, r.v2+10);
            // Create a Pascal string to display the number
            if (i < 10) {
                pstr[0] = 1;           // Length: 1 character
                pstr[1] = '0' + i;     // Digit 0-9
            } else {
                pstr[0] = 2;           // Length: 2 characters
                pstr[1] = '0' + (i / 10);      // Tens (1 for 10-15)
                pstr[2] = '0' + (i % 10);      // Units (0-5 for 10-15)
            }
            DrawString(pstr);
            OffsetRect(&r, 20, 0);
        }
}

void DoText() {
        shroff();
        putchar((char) 12); // Clear screen    
}

// ****************************************************************************
//                              Fonction main
// ****************************************************************************

int main() {
    Model3D* model;
    ObserverParams params;
    char filename[100];
    char input[50];
    int colorpalette = 0; // default color palette
    
newmodel:
    printf("===================================\n");
    printf("       3D OBJ file viewer\n");
    printf("===================================\n\n");
    
    // Creer le modele 3D
    model = createModel3D();
    if (model == NULL) {
        printf("Error: Unable to allocate memory for 3D model\n");
        printf("Press any key to quit...\n");
        keypress();
        return 1;
    }
    // Toujours verrouiller le handle avant d'accéder au pointeur !
   //  test_fill_vertices(&model->vertices); // Uncomment to test memory layout
    // No global HLock/HUnlock needed; handled per array in allocation logic
    
    // Ask for filename
    printf("Enter the filename to read: ");
    if (fgets(filename, sizeof(filename), stdin) != NULL) {
        size_t len = strlen(filename);
        if (len > 0 && filename[len-1] == '\n') {
            filename[len-1] = '\0';
        }
    }
    
    // Charger le modele 3D
    if (loadModel3D(model, filename) < 0) {
        printf("\nError loading file\n");
        printf("Press any key to quit...\n");
        keypress();
        destroyModel3D(model);
        return 1;
    }
    
    // Get observer parameters
    getObserverParams(&params);

    bigloop:
    // Process model with parameters - OPTIMIZED VERSION
    printf("Processing model...\n");
    processModelFast(model, &params, filename);
    
#if ENABLE_DEBUG_SAVE
    // Debug save (WARNING: very slow!)
    saveDebugData(model, "debug.txt");
#endif
    
    // Display information and results
    // displayModelInfo(model);
    // displayResults(model);


    loopReDraw:
    {
        int key = 0;
        char input[50];
        
        if (model->faces.face_count > 0) {
            // Initialize QuickDraw
            startgraph(mode);
            // Draw 3D object
            drawPolygons(model, model->faces.vertex_count, model->faces.face_count, model->vertices.vertex_count);
            // display available colors
            if (colorpalette == 1) { 
                DoColor(); 
            }

            // Wait for key press and get key code
    asm 
        {
        sep #0x20
    loop:
        lda >0xC000     // Read the keyboard status from memory address 0xC000
        bpl loop        // Wait until no key is pressed (= until bit 7 on)
        and #0x007f     // Clear the high bit
        sta >0xC010     // Clear the keypress by writing to 0xC010
        sta key         // Store the key code in variable 'key'
        rep #0x30
        }

    endgraph();        // Close QuickDraw
    
    // CRITICAL: Force cleanup of QuickDraw resources to prevent memory leak
    // Note: InitGraf not available, so we rely on endgraph() cleanup
    // Resource accumulation from FillPoly/FramePoly is inherent to QuickDraw
    
    }  // End of if (model->face_count > 0)
    
    DoText();           // Show text screen


#if ENABLE_DEBUG_SAVE
    sprintf(input, "You pressed key code: %d\n", key);
    printf("%s", input);
#endif


    // Handle keyboard input with switch statement
    switch (key) {
        case 32:  // Space bar - display info and redraw
            // Display some info about model and parameters
            printf("===================================\n");
            printf(" Model information and parameters\n");
            printf("===================================\n");
            printf("Model: %s\n", filename);
            printf("Vertices: %d, Faces: %d\n", model->vertices.vertex_count, model->faces.face_count);
            printf("Observer Parameters:\n");
            printf("    Distance: %.2f\n", FIXED_TO_FLOAT(params.distance));
            printf("    Horizontal Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_h));
            printf("    Vertical Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_v));
            printf("    Screen Rotation Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_w));
            printf("===================================\n");
            printf("\n");
            printf("Press any key to continue...\n");
            keypress();
            goto loopReDraw;
            
        case 65:  // 'A' - decrease distance
        case 97:  // 'a'
            params.distance = params.distance - (params.distance / 10);
            goto bigloop;
            
        case 90:  // 'Z' - increase distance  
        case 122: // 'z'
            params.distance = params.distance + (params.distance / 10);
            goto bigloop;
            
        case 21:  // Right arrow - increase horizontal angle
            params.angle_h = params.angle_h + INT_TO_FIXED(10);
            goto bigloop;
            
        case 8:   // Left arrow - decrease horizontal angle
            params.angle_h = params.angle_h - INT_TO_FIXED(10);
            goto bigloop;
            
        case 10:  // Down arrow - decrease vertical angle
            params.angle_v = params.angle_v - INT_TO_FIXED(10);
            goto bigloop;
            
        case 11:  // Up arrow - increase vertical angle
            params.angle_v = params.angle_v + INT_TO_FIXED(10);
            goto bigloop;
            
        case 87:  // 'W' - increase screen rotation angle
        case 119: // 'w'
            params.angle_w = params.angle_w + INT_TO_FIXED(10);
            goto bigloop;
            
        case 88:  // 'X' - decrease screen rotation angle
        case 120: // 'x'
            params.angle_w = params.angle_w - INT_TO_FIXED(10);
            goto bigloop;
        
        case 67:  // 'C' - toggle color palette display
        case 99:  // 'c'
            colorpalette ^= 1; // Toggle between 0 and 1
            goto loopReDraw;

        case 78:  // 'N' - load new model
        case 110: // 'n'
            destroyModel3D(model);
            goto newmodel;
        
        // dispaly help
        case 72:  // 'H'
        case 104: // 'h'
            printf("===================================\n");
            printf("    HELP - Keyboard Controller\n");
            printf("===================================\n\n");
            printf("Space: Display model info\n");
            printf("A/Z: Increase/Decrease distance\n");
            printf("Arrow Left/Right: Decrease/Increase horizontal angle\n");
            printf("Arrow Up/Down: Increase/Decrease vertical angle\n");
            printf("W/X: Increase/Decrease screen rotation angle\n");
            printf("C: Toggle color palette display\n");
            printf("N: Load new model\n");
            printf("H: Display this help message\n");
            printf("ESC: Quit program\n");
            printf("===================================\n");
            printf("\n");
            printf("Press any key to continue...\n");
            keypress();
            goto loopReDraw;

        case 27:  // ESC - quit
            goto end;
            
        default:  // All other keys - redraw
            goto loopReDraw;
    }
    }  // End of loopReDraw block

    end:
    // Cleanup and exit
    
    // Dispose of the global polygon handle if it was allocated
    if (globalPolyHandle != NULL) {
        if (poly_handle_locked) {
            HUnlock(globalPolyHandle);
        }
        DisposeHandle(globalPolyHandle);
        globalPolyHandle = NULL;
    }
    
    destroyModel3D(model);
    return 0;
}

// Fonction de test pour remplir les tableaux parallèles de vertices
void test_fill_vertices(VertexArrays3D *vtx) {
    printf("[TEST] Remplissage de %d sommets (parallel arrays)...\n", vtx->vertex_count);
    unsigned long addr_x    = (unsigned long)vtx->x;
    unsigned long addr_y    = (unsigned long)vtx->y;
    unsigned long addr_z    = (unsigned long)vtx->z;
    unsigned long addr_xo   = (unsigned long)vtx->xo;
    unsigned long addr_yo   = (unsigned long)vtx->yo;
    unsigned long addr_zo   = (unsigned long)vtx->zo;
    unsigned long addr_x2d  = (unsigned long)vtx->x2d;
    unsigned long addr_y2d  = (unsigned long)vtx->y2d;

    printf("[PTRS] x    = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_x, (unsigned int)((addr_x>>16)&0xFF), (unsigned int)(addr_x&0xFFFF));
    printf("[PTRS] y    = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_y, (unsigned int)((addr_y>>16)&0xFF), (unsigned int)(addr_y&0xFFFF));
    printf("[PTRS] z    = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_z, (unsigned int)((addr_z>>16)&0xFF), (unsigned int)(addr_z&0xFFFF));
    printf("[PTRS] xo   = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_xo, (unsigned int)((addr_xo>>16)&0xFF), (unsigned int)(addr_xo&0xFFFF));
    printf("[PTRS] yo   = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_yo, (unsigned int)((addr_yo>>16)&0xFF), (unsigned int)(addr_yo&0xFFFF));
    printf("[PTRS] zo   = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_zo, (unsigned int)((addr_zo>>16)&0xFF), (unsigned int)(addr_zo&0xFFFF));
    printf("[PTRS] x2d  = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_x2d, (unsigned int)((addr_x2d>>16)&0xFF), (unsigned int)(addr_x2d&0xFFFF));
    printf("[PTRS] y2d  = 0x%06lX (bank=$%02X, offset=$%04X)\n", addr_y2d, (unsigned int)((addr_y2d>>16)&0xFF), (unsigned int)(addr_y2d&0xFFFF));
    keypress();

    for (int i = 0; i < vtx->vertex_count; i++) {
        // Affiche l'adresse longue (24 bits) de l'écriture (bank:offset) pour chaque array
        unsigned long addr_x = (unsigned long)&vtx->x[i];
        unsigned long addr_y = (unsigned long)&vtx->y[i];
        unsigned long addr_z = (unsigned long)&vtx->z[i];
        unsigned int bank_x = (addr_x >> 16) & 0xFF;
        unsigned int offset_x = addr_x & 0xFFFF;
        // printf("i=%d, x=0x%06lX (bank=$%02X, offset=$%04X)", i, addr_x, bank_x, offset_x);
        unsigned int bank_y = (addr_y >> 16) & 0xFF;
        unsigned int offset_y = addr_y & 0xFFFF;
        // printf(", y=0x%06lX (bank=$%02X, offset=$%04X)", addr_y, bank_y, offset_y);
        unsigned int bank_z = (addr_z >> 16) & 0xFF;
        unsigned int offset_z = addr_z & 0xFFFF;

        vtx->x[i] = 1;
        vtx->y[i] = 2;
        vtx->z[i] = 3;
        vtx->xo[i] = 0;
        vtx->yo[i] = 0;
        vtx->zo[i] = 0;
        vtx->x2d[i] = 0;
        vtx->y2d[i] = 0;
    }
    printf("[TEST] Remplissage de %d sommets termine.\n", vtx->vertex_count);
}