/*
 * ============================================================================
 *                              GS3D.CC - SANE Reference Version
 * ============================================================================
 * 
 * 3D rendering program for Apple IIGS with ORCA/C
 * Version 0.4 english - SANE Extended Arithmetic
 * 
 * DESCRIPTION:
 *   This program reads 3D model files in simplified OBJ format,
 *   applies 3D geometric transformations (rotation, translation),
 *   projects the points on a 2D screen, and draws the resulting polygons
 *   using QuickDraw.
 *   
 *   This is the REFERENCE VERSION using Apple IIGS SANE Extended arithmetic
 * 
 * FEATURES:
 *   - Reading OBJ files (vertices "v" and faces "f")
 *   - 3D transformations to viewer system using SANE Extended arithmetic
 *   - Perspective projection on 2D screen
 *   - Graphic rendering with QuickDraw
 *   - Interactive interface to modify parameters
 *   - Performance measurements for comparison with Fixed32 optimized version
 * 
 * ARITHMETIC:
 *   - SANE Extended (80-bit floating point)
 *   - High precision but slower performance
 *   - Reference for Fixed32 optimization comparison
 * 
 * AUTHOR: Bruno
 * DATE: 2025
 * PLATFORM: Apple IIGS - ORCA/C with SANE numerics
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

// ============================================================================
//                            GLOBAL CONSTANTS
// ============================================================================

// Performance and debug configuration
#define ENABLE_DEBUG_SAVE 0     // 1 = Enable debug save (SLOW!), 0 = Disable
//#define PERFORMANCE_MODE 0      // 1 = Optimized performance mode, 0 = Debug mode
// OPTIMIZATION: Performance mode - disable printf
#define PERFORMANCE_MODE 0      // 1 = no printf, 0 = normal printf

#define MAX_LINE_LENGTH 256     // Maximum file line size
#define MAX_VERTICES 1000       // Maximum vertices in a 3D model
#define MAX_FACES 1000          // Maximum faces in a 3D model
#define MAX_FACE_VERTICES 20    // Maximum vertices per face (polygon)
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
typedef struct {
    Extended x, y, z;       // Original coordinates from OBJ file (SANE)
    Extended xo, yo, zo;    // Transformed coordinates (observer system)
    int x2d, y2d;           // Projected coordinates on 2D screen (pixels)
} Vertex3D;

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
 *   vertex_indices  : Array of vertex indices (1-based numbering
 *                     as in the standard OBJ format)
 * 
 * NOTES:
 *   - Indices are stored in base 1 (first vertex = index 1)
 *   - Conversion to base 0 needed to access the C array
 *   - Maximum MAX_FACE_VERTICES vertices per face to prevent overflow
 */
typedef struct {
    int vertex_count;                           // Number of vertices in the face
    int vertex_indices[MAX_FACE_VERTICES];     // Vertex indices (base 1)
    Extended z_max;                            // Maximum depth of the face (for sorting, SANE)
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
    Extended angle_h;   // Observer horizontal angle (Y rotation, SANE)
    Extended angle_v;   // Observer vertical angle (X rotation, SANE) 
    Extended angle_w;   // 2D projection rotation angle (SANE)
    Extended distance;  // Observer-object distance (perspective, SANE)
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
    Vertex3D *vertices;     // Dynamic vertex array of the model
    Face3D *faces;          // Dynamic face array of the model
    int vertex_count;       // Actual number of loaded vertices
    int face_count;         // Actual number of loaded faces
} Model3D;

// ============================================================================
//                       FUNCTION DECLARATIONS
// ============================================================================

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
int readVertices(const char* filename, Vertex3D* vertices, int max_vertices);

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
int readFaces(const char* filename, Face3D* faces, int max_faces);

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
void transformToObserver(Vertex3D* vertices, int vertex_count, 
                        Extended angle_h, Extended angle_v, Extended distance);

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
void projectTo2D(Vertex3D* vertices, int vertex_count, Extended angle_w);

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
void drawPolygons(Vertex3D* vertices, Face3D* faces, int face_count, int vertex_count);
void calculateFaceDepths(Vertex3D* vertices, Face3D* faces, int face_count);
void sortFacesByDepth(Face3D* faces, int face_count);
void sortFacesByDepth_insertion(Face3D* faces, int face_count);
void sortFacesByDepth_quicksort(Face3D* faces, int low, int high);
void sortFacesByDepth_insertion_range(Face3D* faces, int low, int high);
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
void processModelFast(Model3D* model, ObserverParams* params);

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
        return NULL;  // Main structure allocation failed
    }
    
    // Step 2: Vertex array allocation
    // Size: MAX_VERTICES * sizeof(Vertex3D) bytes
    model->vertices = (Vertex3D*)malloc(MAX_VERTICES * sizeof(Vertex3D));
    if (model->vertices == NULL) {
        free(model);  // Cleanup: free main structure
        return NULL;  // Vertex array allocation failed
    }
    
    // Step 3: Face array allocation
    // Size: MAX_FACES * sizeof(Face3D) bytes
    model->faces = (Face3D*)malloc(MAX_FACES * sizeof(Face3D));
    if (model->faces == NULL) {
        free(model->vertices);  // Cleanup: free vertex array
        free(model);            // Cleanup: free main structure
        return NULL;            // Face array allocation failed
    }
    
    // Step 4: Counter initialization
    model->vertex_count = 0;    // No vertices loaded initially
    model->face_count = 0;      // No faces loaded initially
    
    return model;  // Success: return initialized model
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
    // Check main pointer
    if (model != NULL) {
        // Free vertex array (if allocated)
        if (model->vertices != NULL) {
            free(model->vertices);
        }
        
        // Free face array (if allocated)
        if (model->faces != NULL) {
            free(model->faces);
        }
        
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
    model->vertex_count = readVertices(filename, model->vertices, MAX_VERTICES);
    if (model->vertex_count < 0) {
        return -1;  // Critical failure: unable to read vertices
    }
    
    // Step 2: Read faces from OBJ file
    model->face_count = readFaces(filename, model->faces, MAX_FACES);
    if (model->face_count < 0) {
        // Critical failure: unable to read vertices
        printf("\nWarning: Unable to read faces\n");
        model->face_count = 0;  // No faces available
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
 * - Automatic string->Extended conversion with atof() then cast
 */
void getObserverParams(ObserverParams* params) {
    char input[50];  // Buffer for user input
    
    // Display section header
    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values)\n");
    
    // Input horizontal angle (rotation around Y)
    printf("Horizontal angle (degrees, default 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_h = 30.0;     // Default value if ENTER (SANE)
        } else {
            params->angle_h = (Extended)atof(input);  // String->Extended conversion (SANE)
        }
    } else {
        params->angle_h = 30.0;         // Default value if error (SANE)
    }
    
    // Input vertical angle (rotation around X)  
    printf("Vertical angle (degrees, default 15): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_v = 15.0;     // Default value if ENTER (SANE)
        } else {
            params->angle_v = (Extended)atof(input);  // String->Extended conversion (SANE)
        }
    } else {
        params->angle_v = 15.0;         // Default value if error (SANE)
    }
    
    // Input observation distance (zoom/perspective)
    printf("Distance (default 10): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->distance = 10.0;    // Default value if ENTER (SANE)
        } else {
            params->distance = (Extended)atof(input); // String->Extended conversion (SANE)
        }
    } else {
        params->distance = 10.0;        // Default distance: balanced view (SANE)
    }
    
    // Input screen rotation angle (final 2D rotation)
    printf("Screen rotation angle (degrees, default 0): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_w = 0.0;      // Default value if ENTER (SANE)
        } else {
            params->angle_w = (Extended)atof(input);  // String->Extended conversion (SANE)
        }
    } else {
        params->angle_w = 0.0;          // No rotation by default (SANE)
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
    printf("\nAnalysis summary:\n");
    printf("====================\n");
    printf("Number of vertices (3D points) found: %d\n", model->vertex_count);
    printf("Number of faces found: %d\n", model->face_count);
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
    
    // Display vertices if present
    if (model->vertex_count > 0) {
        printf("\nComplete coordinates (Original -> 3D -> 2D):\n");
        printf("-----------------------------------------------\n");
        
        // Go through all vertices
        for (i = 0; i < model->vertex_count; i++) {
            // Check if vertex is visible on screen
            if (model->vertices[i].x2d >= 0 && model->vertices[i].y2d >= 0) {
                // Visible vertex: display all coordinates
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (%d,%d)\n", 
                       i + 1,  // Base-1 numbering for user
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,     // Original
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo,  // Transformed
                       model->vertices[i].x2d, model->vertices[i].y2d);                      // Projected 2D
            } else {
                // Invisible vertex (behind observer or off screen)
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (invisible)\n", 
                       i + 1,  // Base-1 numbering for user
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,     // Original
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo); // Transformed
            }
        }
    }
    
    // Display faces if present
    if (model->face_count > 0) {
        printf("\nFace list:\n");
        printf("----------------\n");
        
        // Go through all faces
        for (i = 0; i < model->face_count; i++) {
            printf("  Face %3d (%d vertices, z_max=%.2f): ", i + 1, model->faces[i].vertex_count, model->faces[i].z_max);
            
            // Display vertex indices of this face
            for (j = 0; j < model->faces[i].vertex_count; j++) {
                printf("%d", model->faces[i].vertex_indices[j]); // Index base 1
                if (j < model->faces[i].vertex_count - 1) printf("-"); // Separator
            }
            printf("\n");
            
            // Detailed display of vertex coordinates of this face
            printf("       Coordinates of vertices of this face:\n");
            for (j = 0; j < model->faces[i].vertex_count; j++) {
                int vertex_idx = model->faces[i].vertex_indices[j] - 1; // Convert base 1 to base 0
                if (vertex_idx >= 0 && vertex_idx < model->vertex_count) {
                    printf("         Vertex %d: (%.2f,%.2f,%.2f) -> (%d,%d)\n",
                           model->faces[i].vertex_indices[j], // Original index base 1
                           model->vertices[vertex_idx].x, model->vertices[vertex_idx].y, model->vertices[vertex_idx].z,
                           model->vertices[vertex_idx].x2d, model->vertices[vertex_idx].y2d);
                } else {
                    printf("         Vertex %d: ERROR - Index out of bounds!\n", 
                           model->faces[i].vertex_indices[j]);
                }
            }
            printf("\n");
        }
        
        // Launch graphic rendering (if faces available)
        drawPolygons(model->vertices, model->faces, model->face_count, model->vertex_count);
    }
}

/**
 * ULTRA-FAST FUNCTION: Combined Transformation + Projection
 * ==========================================================
 */
void processModelFast(Model3D* model, ObserverParams* params) {
    int i;
    Extended rad_h, rad_v, rad_w;
    Extended cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Extended x, y, z, zo, xo, yo;
    Extended inv_zo, x2d_temp, y2d_temp;
    
    // Pre-calculate ALL trigonometric values (SANE optimized)
    rad_h = params->angle_h * PI / 180.0;
    rad_v = params->angle_v * PI / 180.0;
    rad_w = params->angle_w * PI / 180.0;
    
    cos_h = cos(rad_h);
    sin_h = sin(rad_h);
    cos_v = cos(rad_v);
    sin_v = sin(rad_v);
    cos_w = cos(rad_w);
    sin_w = sin(rad_w);
    
    // Pre-calculate all trigonometric products
    const Extended cos_h_cos_v = cos_h * cos_v;
    const Extended sin_h_cos_v = sin_h * cos_v;
    const Extended cos_h_sin_v = cos_h * sin_v;
    const Extended sin_h_sin_v = sin_h * sin_v;
    const Extended scale = 100.0;
    const Extended centre_x_f = (Extended)CENTRE_X;
    const Extended centre_y_f = (Extended)CENTRE_Y;
    const Extended distance = params->distance;
    
    // Mesure de performance pour la transformation + projection
    long start_transform_ticks = GetTick();
    

    
    // Boucle unique transformation + projection
    for (i = 0; i < model->vertex_count; i++) {
        x = model->vertices[i].x;
        y = model->vertices[i].y;
        z = model->vertices[i].z;
        
        // Transformation 3D
        zo = -x * cos_h_cos_v - y * sin_h_cos_v - z * sin_v + distance;
        

        
        if (zo > 0.0f) {
            xo = -x * sin_h + y * cos_h;
            yo = -x * cos_h_sin_v - y * sin_h_sin_v + z * cos_v;
            
            // Store for face sorting
            model->vertices[i].zo = zo;
            model->vertices[i].xo = xo;
            model->vertices[i].yo = yo;
            
            // Projection 2D directe
            inv_zo = scale / zo;
            x2d_temp = xo * inv_zo + centre_x_f;
            y2d_temp = centre_y_f - yo * inv_zo;
            
            // Rotation finale
            model->vertices[i].x2d = (int)(cos_w * (x2d_temp - centre_x_f) - sin_w * (centre_y_f - y2d_temp) + centre_x_f);
            model->vertices[i].y2d = (int)(centre_y_f - (sin_w * (x2d_temp - centre_x_f) + cos_w * (centre_y_f - y2d_temp)));
        } else {
            model->vertices[i].zo = zo;
            model->vertices[i].xo = 0;
            model->vertices[i].yo = 0;
            model->vertices[i].x2d = -1;
            model->vertices[i].y2d = -1;
        }
    }
    
    long end_transform_ticks = GetTick();
    
    // Face sorting after transformation
    long start_calc_ticks = GetTick();
    calculateFaceDepths(model->vertices, model->faces, model->face_count);
    long end_calc_ticks = GetTick();
    
    long start_sort_ticks = GetTick();
    sortFacesByDepth(model->faces, model->face_count);
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
int readVertices(const char* filename, Vertex3D* vertices, int max_vertices) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s'\n", filename);
        printf("Check that the file exists and you have read permissions.\n");
        return -1;  // Return -1 on error
    }
    
    printf("\nFile contents '%s':\n", filename);
    printf("========================\n\n");
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // printf("%3d: %s", line_number, line);
        
        // Check if line starts with "v " (vertex - standard OBJ format)
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;  // Temporary reading in float
                // Extract coordinates x, y, z
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    // Convert to Extended to use SANE
                    vertices[vertex_count].x = (Extended)x;
                    vertices[vertex_count].y = (Extended)y;
                    vertices[vertex_count].z = (Extended)z;
                    vertex_count++;
                    // printf("     -> Vertex %d: (%.3f, %.3f, %.3f)\n", 
                    //        vertex_count, x, y, z);
                }
            } else {
                printf("     -> WARNING: Vertex limit reached (%d)\n", max_vertices);
            }
        }
        
        line_number++;
    }
    
    // Close file
    fclose(file);
    
    // printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
    return vertex_count;  // Return the number of vertices read
}

// Function to read faces from a 3D file
int readFaces(const char* filename, Face3D* faces, int max_faces) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    char *token;
    int line_number = 1;
    int face_count = 0;
    int i;
    
    // Open file in read mode
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error: Unable to open file '%s' to read faces\n", filename);
        return -1;  // Return -1 on error
    }
    
    printf("\nReading faces from file '%s':\n", filename);
    printf("==================================\n\n");
    
    // Read file line by line
    while (fgets(line, sizeof(line), file) != NULL) {
        // Check if line starts with "f " (face)
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < max_faces) {
                // printf("%3d: %s", line_number, line);
                
                faces[face_count].vertex_count = 0;
                faces[face_count].display_flag = 1;  // Initialize as displayable by default
                
                // Use a more robust approach than strtok
                char *ptr = line + 2;  // Start after "f "
                faces[face_count].vertex_count = 0;
                
                // Analyze character by character
                while (*ptr != '\0' && *ptr != '\n' && faces[face_count].vertex_count < MAX_FACE_VERTICES) {
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
                    
                    // Verify that index is valid (base 1, so >= 1)
                    if (vertex_index >= 1) {
                        faces[face_count].vertex_indices[faces[face_count].vertex_count] = vertex_index;
                        faces[face_count].vertex_count++;
                        // printf("  -> Vertex index read: %d\n", vertex_index);
                    } else if (vertex_index > 0) {  // vertex_index == 0 means no number was read
                        printf("     -> WARNING: Invalid vertex index %d ignored\n", vertex_index);
                    }
                }
                
                // printf("     -> Face %d: %d vertices (", face_count + 1, faces[face_count].vertex_count);
                // for (i = 0; i < faces[face_count].vertex_count; i++) {
                //     printf("%d", faces[face_count].vertex_indices[i]);
                //     if (i < faces[face_count].vertex_count - 1) printf(",");
                // }
                // printf(")\n");
                
                // Additional validation: if vertex count = 0, ignore this face
                if (faces[face_count].vertex_count == 0) {
                    printf("     -> WARNING: Face without valid vertices ignored\n");
                } else {
                    face_count++;
                }
            } else {
                printf("     -> WARNING: Face limit reached (%d)\n", max_faces);
            }
        }
        
        line_number++;
    }
    
    // Close file
    fclose(file);
    
    // printf("\n\nFace analysis complete. %d faces read.\n", face_count);
    return face_count;  // Return number of faces read
}

// Function to transform coordinates to observer system
void transformToObserver(Vertex3D* vertices, int vertex_count, 
                        Extended angle_h, Extended angle_v, Extended distance) {
    int i;
    Extended rad_h, rad_v;
    Extended cos_h, sin_h, cos_v, sin_v;
    Extended x, y, z;
    
    // Convert angles to radians (SANE)
    rad_h = angle_h * PI / 180.0;
    rad_v = angle_v * PI / 180.0;
    
    // Pre-calculate trigonometric values
    cos_h = cos(rad_h);
    sin_h = sin(rad_h);
    cos_v = cos(rad_v);
    sin_v = sin(rad_v);
    
    // OPTIMIZATION: Pre-calculate trigonometric products (SANE)
    Extended cos_h_cos_v = cos_h * cos_v;
    Extended sin_h_cos_v = sin_h * cos_v;
    Extended cos_h_sin_v = cos_h * sin_v;
    Extended sin_h_sin_v = sin_h * sin_v;
    
#if !PERFORMANCE_MODE
    printf("\nTransformation to observer system:\n");
    printf("Horizontal angle: %.1f degrees\n", angle_h);
    printf("Vertical angle: %.1f degrees\n", angle_v);
    printf("Distance: %.3f\n", distance);
    printf("==========================================\n");
#endif
    
    // Transform each vertex - OPTIMIZED VERSION
    for (i = 0; i < vertex_count; i++) {
        // Access coordinates directly
        x = vertices[i].x;
        y = vertices[i].y;
        z = vertices[i].z;
        
        // OPTIMIZATION: Use pre-calculated products
        vertices[i].zo = -x * cos_h_cos_v - y * sin_h_cos_v - z * sin_v + distance;
        vertices[i].xo = -x * sin_h + y * cos_h;
        vertices[i].yo = -x * cos_h_sin_v - y * sin_h_sin_v + z * cos_v;
        
        // printf("Vertex %3d: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", 
        //        i + 1, x, y, z, vertices[i].xo, vertices[i].yo, vertices[i].zo);
    }
}

// Function to project 3D coordinates onto 2D screen
void projectTo2D(Vertex3D* vertices, int vertex_count, Extended angle_w) {
    int i;
    Extended rad_w;
    Extended cos_w, sin_w;
    Extended x2d_temp, y2d_temp, save_x;
    
    // Convert angle to radians (SANE)
    rad_w = angle_w * PI / 180.0;
    
    // Pre-calculate trigonometric values (SANE)
    cos_w = cos(rad_w);
    sin_w = sin(rad_w);
    
    // OPTIMIZATION: Pre-calculate constants (SANE)
    const Extended scale = 100.0;
    const Extended centre_x_f = (Extended)CENTRE_X;
    const Extended centre_y_f = (Extended)CENTRE_Y;
    
#if !PERFORMANCE_MODE
    printf("\nProjection on 2D screen:\n");
    printf("Rotation angle: %.1f degrees\n", angle_w);
    printf("Screen center: (%d, %d)\n", CENTRE_X, CENTRE_Y);
    printf("===========================\n");
#endif
    
    // Project each vertex - OPTIMIZED VERSION
    for (i = 0; i < vertex_count; i++) {
        // OPTIMIZATION: Visibility test first
        if (vertices[i].zo > 0.0) {
            // OPTIMIZATION: Single division per vertex (SANE)
            Extended inv_zo = scale / vertices[i].zo;
            
            // Optimized perspective projection
            x2d_temp = vertices[i].xo * inv_zo + centre_x_f;
            y2d_temp = centre_y_f - vertices[i].yo * inv_zo;
            
            // OPTIMIZATION: Rotation without temporary variable
            vertices[i].x2d = (int)(cos_w * (x2d_temp - centre_x_f) - sin_w * (centre_y_f - y2d_temp) + centre_x_f);
            vertices[i].y2d = (int)(centre_y_f - (sin_w * (x2d_temp - centre_x_f) + cos_w * (centre_y_f - y2d_temp)));
            
            // printf("Vertex %3d: 3D(%.2f,%.2f,%.2f) -> 2D(%d,%d)\n", 
            //        i + 1, vertices[i].xo, vertices[i].yo, vertices[i].zo, 
            //        vertices[i].x2d, vertices[i].y2d);
        } else {
            // Point behind observer, no projection
            vertices[i].x2d = -1;
            vertices[i].y2d = -1;

        }
    }
}

/**
 * CALCULATING MAXIMUM FACE DEPTHS AND VISIBILITY FLAGS
 * =====================================================
 * 
 * This function calculates for each face:
 * 1. The maximum depth (z_max) of all its vertices in the observer coordinate system
 * 2. The display visibility flag based on vertex positions relative to camera
 * 
 * The z_max value is used for face sorting during rendering (painter's algorithm).
 * The display_flag is used to cull faces that have vertices behind the camera.
 * 
 * PARAMETERS:
 *   vertices   : Array of vertices with coordinates in observer system
 *   faces      : Array of faces to process  
 *   face_count : Number of faces
 * 
 * ALGORITHM:
 *   For each face:
 *   - Initialize z_max with very small value (-9999.0)
 *   - Initialize display_flag as true (displayable)
 *   - For each vertex of the face:
 *     * Check if vertex is behind camera (zo <= 0)
 *     * If ANY vertex is behind camera, set display_flag = false
 *     * Update z_max with maximum zo value found
 *   - Store both z_max and display_flag in the face structure
 * 
 * CULLING LOGIC:
 *   - If ANY vertex has zo <= 0, the entire face is marked as non-displayable
 *   - This prevents rendering artifacts from perspective projection errors
 *   - Improves performance by eliminating faces early in the pipeline
 * 
 * NOTES:
 *   - Must be called AFTER transformToObserver() or processModelFast()
 *   - Uses zo coordinates (observer system depth)
 *   - Higher z_max value means face is farther away
 *   - display_flag = 1 means visible, 0 means hidden (behind camera)
 */
void calculateFaceDepths(Vertex3D* vertices, Face3D* faces, int face_count) {
    int i, j;
    
    // For each face
    for (i = 0; i < face_count; i++) {
        Extended z_max = -9999.0;  // Initialize with very small value (SANE)
        int display_flag = 1;      // Initialize as displayable (true)
        
        // Go through all vertices of this face
        for (j = 0; j < faces[i].vertex_count; j++) {
            int vertex_idx = faces[i].vertex_indices[j] - 1; // Convert base 1 to base 0
            
            // Verify that index is valid (use reasonable number of vertices)
            if (vertex_idx >= 0) {
                // Check if vertex is behind camera (zo <= 0)
                if (vertices[vertex_idx].zo <= 0.0) {
                    display_flag = 0;  // Don't display this face
                }
                
                // Compare with zo coordinate (depth in observer system)
                if (vertices[vertex_idx].zo > z_max) {
                    z_max = vertices[vertex_idx].zo;
                }
            }
        }
        
        // Store maximum depth and display flag in the face
        faces[i].z_max = z_max;
        faces[i].display_flag = display_flag;
        
        // Optional debug

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
void sortFacesByDepth(Face3D* faces, int face_count) {
    int i;
    
    // Cas trivial: 0 ou 1 face
    if (face_count <= 1) {
        return;
    }
    
    // Optimization 1: Check if array is already sorted (common case in 3D)
    int already_sorted = 1;
    for (i = 0; i < face_count - 1; i++) {
        if (faces[i].z_max < faces[i + 1].z_max) {
            already_sorted = 0;
            break;
        }
    }
    if (already_sorted) {
        // printf("Array already sorted - optimization enabled\n");
        return;  // Already sorted, no work to do
    }
    
    // Optimization 2: Algorithmic choice based on size
    if (face_count <= 10) {
        // Insertion sort optimized for small collections
        printf("Insertion sort (small collection: %d faces)\n", face_count);
        sortFacesByDepth_insertion(faces, face_count);
    } else {
        // Quick sort for large collections
        printf("Quick sort (large collection: %d faces)\n", face_count);
        sortFacesByDepth_quicksort(faces, 0, face_count - 1);
    }
    

}

/**
 * TRI PAR INSERTION OPTIMISE (pour petites collections)
 * =====================================================
 */
void sortFacesByDepth_insertion(Face3D* faces, int face_count) {
    int i, j;
    Face3D temp_face;
    
    for (i = 1; i < face_count; i++) {
        // Sauvegarder la face courante
        temp_face = faces[i];
        
        // Decaler les faces avec z_max plus petit vers la droite
        // (tri décroissant: face la plus lointaine en premier)
        j = i - 1;
        while (j >= 0 && faces[j].z_max < temp_face.z_max) {
            faces[j + 1] = faces[j];
            j--;
        }
        
        // Inserer la face courante a sa position
        faces[j + 1] = temp_face;
    }
}

/**
 * TRI RAPIDE OPTIMISE (pour grandes collections)
 * ==============================================
 */
void sortFacesByDepth_quicksort(Face3D* faces, int low, int high) {
    if (low < high) {
        // Optimisation: basculer vers tri par insertion pour petites partitions
        if (high - low + 1 <= 8) {
            sortFacesByDepth_insertion_range(faces, low, high);
            return;
        }
        
        // Partition avec median-de-trois pour ameliorer les performances
        int pivot_index = partition_median3(faces, low, high);
        
        // Recursion sur les deux partitions
        sortFacesByDepth_quicksort(faces, low, pivot_index - 1);
        sortFacesByDepth_quicksort(faces, pivot_index + 1, high);
    }
}

/**
 * TRI PAR INSERTION SUR UNE PLAGE (helper pour quicksort)
 * =======================================================
 */
void sortFacesByDepth_insertion_range(Face3D* faces, int low, int high) {
    int i, j;
    Face3D temp_face;
    
    for (i = low + 1; i <= high; i++) {
        if (faces[i].z_max <= faces[i - 1].z_max) {
            continue;  // Deja bien place
        }
        
        temp_face = faces[i];
        j = i - 1;
        while (j >= low && faces[j].z_max < temp_face.z_max) {
            faces[j + 1] = faces[j];
            j--;
        }
        faces[j + 1] = temp_face;
    }
}

/**
 * PARTITION AVEC MEDIAN-DE-TROIS (pour quicksort stable)
 * ======================================================
 */
int partition_median3(Face3D* faces, int low, int high) {
    int mid = low + (high - low) / 2;
    Face3D temp;
    
    // Median-de-trois: organiser faces[low], faces[mid], faces[high]
    if (faces[mid].z_max > faces[high].z_max) {
        temp = faces[mid]; faces[mid] = faces[high]; faces[high] = temp;
    }
    if (faces[low].z_max > faces[high].z_max) {
        temp = faces[low]; faces[low] = faces[high]; faces[high] = temp;
    }
    if (faces[mid].z_max > faces[low].z_max) {
        temp = faces[mid]; faces[mid] = faces[low]; faces[low] = temp;
    }
    
    // Utiliser faces[low] comme pivot (median des trois)
    Extended pivot = faces[low].z_max;
    int i = low;
    int j = high + 1;
    
    while (1) {
        // Trouver element a gauche plus petit que pivot
        do {
            i++;
        } while (i <= high && faces[i].z_max > pivot);
        
        // Trouver element a droite plus grand que pivot
        do {
            j--;
        } while (faces[j].z_max < pivot);
        
        if (i >= j) break;
        
        // Echanger faces[i] et faces[j]
        temp = faces[i];
        faces[i] = faces[j];
        faces[j] = temp;
    }
    
    // Placer le pivot a sa position finale
    temp = faces[low];
    faces[low] = faces[j];
    faces[j] = temp;
    
    return j;
}

// Function to draw polygons with QuickDraw
void drawPolygons(Vertex3D* vertices, Face3D* faces, int face_count, int vertex_count) {
    int i, j;
    Handle polyHandle;
    DynamicPolygon *poly;
    int min_x, max_x, min_y, max_y;
    int valid_faces_drawn = 0;
    int invalid_faces_skipped = 0;
    int triangle_count = 0;
    int quad_count = 0;
    Pattern pat;
  
    SetPenMode(0);
    // Draw each face
    for (i = 0; i < face_count; i++) {
        // Check if face should be displayed (not behind camera)
        if (faces[i].display_flag == 0) {
            continue;  // Skip faces with vertices behind camera (zo <= 0)
        }
        
        // Draw all faces with at least 3 vertices
        if (faces[i].vertex_count >= 3) {
            
            // Calculate polygon size (header + bbox + points)
            int polySize = 2 + 8 + (faces[i].vertex_count * 4);
            
            // Allocate memory for the polygon
            polyHandle = NewHandle((long)polySize, userid(), 0xC015, 0L);
            if (polyHandle != NULL) {
                HLock(polyHandle);
                poly = (DynamicPolygon *)*polyHandle;
                
                // Fill polygon structure
                poly->polySize = polySize;
                
                // Initialize bounding box limits
                min_x = max_x = min_y = max_y = -1;
                
                // Copy points and calculate bounding box
                for (j = 0; j < faces[i].vertex_count; j++) {
                    int vertex_idx = faces[i].vertex_indices[j] - 1; // Convert base 1 to base 0
                    
                    poly->polyPoints[j].h = mode / 320 * vertices[vertex_idx].x2d;
                    poly->polyPoints[j].v = vertices[vertex_idx].y2d;
                    
                    // Update bounding box
                    if (min_x == -1 || vertices[vertex_idx].x2d < min_x) min_x = vertices[vertex_idx].x2d;
                    if (max_x == -1 || vertices[vertex_idx].x2d > max_x) max_x = vertices[vertex_idx].x2d;
                    if (min_y == -1 || vertices[vertex_idx].y2d < min_y) min_y = vertices[vertex_idx].y2d;
                    if (max_y == -1 || vertices[vertex_idx].y2d > max_y) max_y = vertices[vertex_idx].y2d;
                }
                
                // Define bounding box
                poly->polyBBox.h1 = min_x;
                poly->polyBBox.v1 = min_y;
                poly->polyBBox.h2 = max_x;
                poly->polyBBox.v2 = max_y;
                
                // Define color (cyclic to vary colors)
                //SetSolidPenPat((i % 15) + 1);

                // Draw the polygon
                SetSolidPenPat(14);     // light gray
                GetPenPat(pat);
                FillPoly(polyHandle,pat);
                SetSolidPenPat(7);      // red
                FramePoly(polyHandle);
                
                // Cleanup
                HUnlock(polyHandle);
                DisposeHandle(polyHandle);
                
                valid_faces_drawn++;
            } else {
                // printf("Error: Cannot allocate memory for face %d\n", i + 1);
                invalid_faces_skipped++;
            }
        } else {
            // printf("Face %d ignored (less than 3 vertices: %d)\n", i + 1, faces[i].vertex_count);
            invalid_faces_skipped++;
        }
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
    
    // Open debug file for writing
    debug_file = fopen(debug_filename, "w");
    if (debug_file == NULL) {
        printf("Error: Unable to create debug file '%s'\n", debug_filename);
        return;
    }
    

    
    // File header
    fprintf(debug_file, "=== 3D MODEL DEBUG DATA ===\n");
    fprintf(debug_file, "Generation date: %s", __DATE__);
    fprintf(debug_file, "\n\n");
    
    // General statistics
    fprintf(debug_file, "=== STATISTICS ===\n");
    fprintf(debug_file, "Loaded vertices: %d\n", model->vertex_count);
    fprintf(debug_file, "Loaded faces: %d\n", model->face_count);
    fprintf(debug_file, "\n");
    
    // Face analysis by vertex count
    int triangle_count = 0, quad_count = 0, other_count = 0;
    for (i = 0; i < model->face_count; i++) {
        if (model->faces[i].vertex_count == 3) triangle_count++;
        else if (model->faces[i].vertex_count == 4) quad_count++;
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
    for (i = 0; i < model->vertex_count; i++) {
        fprintf(debug_file, "V%03d | %8.3f %8.3f %8.3f | %4d %4d\n", 
                i + 1,
                model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,
                model->vertices[i].x2d, model->vertices[i].y2d);
    }
    fprintf(debug_file, "\n");
    
    // Complete face list
    fprintf(debug_file, "=== FACES ===\n");
    for (i = 0; i < model->face_count; i++) {
        fprintf(debug_file, "Face F%03d (%d vertices):\n", i + 1, model->faces[i].vertex_count);
        fprintf(debug_file, "  Indices: ");
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            fprintf(debug_file, "V%d", model->faces[i].vertex_indices[j]);
            if (j < model->faces[i].vertex_count - 1) fprintf(debug_file, ", ");
        }
        fprintf(debug_file, "\n");
        
        // 3D and 2D coordinates of each vertex of the face
        fprintf(debug_file, "  Coordinates:\n");
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            int vertex_idx = model->faces[i].vertex_indices[j] - 1; // Convert base-1 to base-0
            if (vertex_idx >= 0 && vertex_idx < model->vertex_count) {
                fprintf(debug_file, "    V%d: 3D(%.3f, %.3f, %.3f) -> 2D(%d, %d)\n",
                        model->faces[i].vertex_indices[j],
                        model->vertices[vertex_idx].x, model->vertices[vertex_idx].y, model->vertices[vertex_idx].z,
                        model->vertices[vertex_idx].x2d, model->vertices[vertex_idx].y2d);
            } else {
                fprintf(debug_file, "    V%d: ERROR - Index out of bounds!\n", model->faces[i].vertex_indices[j]);
            }
        }
        fprintf(debug_file, "\n");
    }
    
    // Integrity check
    fprintf(debug_file, "=== INTEGRITY CHECK ===\n");
    int errors = 0;
    for (i = 0; i < model->face_count; i++) {
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            int vertex_idx = model->faces[i].vertex_indices[j] - 1;
            if (vertex_idx < 0 || vertex_idx >= model->vertex_count) {
                fprintf(debug_file, "ERROR: Face F%d references non-existent vertex V%d (index %d out of bounds [1-%d])\n",
                        i + 1, model->faces[i].vertex_indices[j], vertex_idx + 1, model->vertex_count);
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
        unsigned char pstr[4];  // Pascal string: [length][characters...]

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
    processModelFast(model, &params);
    
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
        
        if (model->face_count > 0) {
            // Initialize QuickDraw
            startgraph(mode);
            // Draw 3D object
            drawPolygons(model->vertices, model->faces, model->face_count, model->vertex_count);
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
    }  // End of if (model->face_count > 0)
    
    DoText();           // Show text screen





    // Handle keyboard input with switch statement
    switch (key) {
        case 32:  // Space bar - display info and redraw
            // Display some info about model and parameters
            printf("===================================\n");
            printf(" Model information and parameters\n");
            printf("===================================\n");
            printf("Model: %s\n", filename);
            printf("Vertices: %d, Faces: %d\n", model->vertex_count, model->face_count);
            printf("Observer Parameters:\n");
            printf("    Distance: %.2f\n", params.distance);
            printf("    Horizontal Angle: %.1f\n", params.angle_h);
            printf("    Vertical Angle: %.1f\n", params.angle_v);
            printf("    Screen Rotation Angle: %.1f\n", params.angle_w);
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
            params.angle_h = params.angle_h + 10;
            goto bigloop;
            
        case 8:   // Left arrow - decrease horizontal angle
            params.angle_h = params.angle_h - 10;
            goto bigloop;
            
        case 10:  // Down arrow - decrease vertical angle
            params.angle_v = params.angle_v - 10;
            goto bigloop;
            
        case 11:  // Up arrow - increase vertical angle
            params.angle_v = params.angle_v + 10;
            goto bigloop;
            
        case 87:  // 'W' - increase screen rotation angle
        case 119: // 'w'
            params.angle_w = params.angle_w + 10;
            goto bigloop;
            
        case 88:  // 'X' - decrease screen rotation angle
        case 120: // 'x'
            params.angle_w = params.angle_w - 10;
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
    destroyModel3D(model);
    return 0;
}