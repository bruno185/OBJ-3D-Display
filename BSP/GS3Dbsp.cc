/*
 * GS3Dbsp.CC - Version BSP du viewer 3D Apple IIGS
 *
 * Copie de GS3Df.cc, adaptée pour charger et afficher un modèle 3D
 * au format BSP binaire (exporté par export_bsp_bin.py).
 *
 * L'interface utilisateur et les structures sont modifiées au minimum.
 *
 * Copyright 2025 Bruno
 */

// ============================================================================
//  INCLUDES
// ============================================================================

#include <stdio.h>      // Standard input/output (printf, fgets, etc.)
#include <asm.h>        // ORCA specific assembler functions
#include <string.h>     // String manipulation (strlen, strcmp, etc.)
#include <misctool.h>   // ORCA misc tools (GetTick, keypress, etc.)
#include <stdlib.h>     // Standard functions (alloc, free, atof, etc.)
#include <math.h>       // Math functions (cos, sin, sqrt, etc.)
#include <quickdraw.h>  // Apple IIGS QuickDraw graphics API
#include <event.h>      // System event management
#include <memory.h>     // Advanced memory management (NewHandle, etc.)
#include <window.h>     // Window management
#include <orca.h>       // ORCA specific functions (startgraph, etc.)

// ============================================================================
//  TYPES ET MACROS - VIRGULE FIXE
// ============================================================================

typedef long Fixed32;
typedef long long Fixed64;

#define FIXED_SHIFT     16
#define FIXED_SCALE     (1L << FIXED_SHIFT)  // 65536
#define FIXED_MASK      (FIXED_SCALE - 1)
#define FIXED_HALF      (FIXED_SCALE >> 1)

// Constantes mathématiques en virgule fixe
#define FIXED_PI        205887L               // PI en 16.16
#define FIXED_2PI       411775L               // 2*PI en 16.16
#define FIXED_PI_2      102944L               // PI/2 en 16.16
#define FIXED_ONE       FIXED_SCALE

// Macros de conversion
#define INT_TO_FIXED(x)     ((Fixed32)(x) << FIXED_SHIFT)
#define FIXED_TO_INT(x)     ((int)((x) >> FIXED_SHIFT))
#define FLOAT_TO_FIXED(x)   ((Fixed32)((x) * FIXED_SCALE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / (float)FIXED_SCALE)

// Opérations arithmétiques
#define FIXED_ADD(a, b)     ((a) + (b))
#define FIXED_SUB(a, b)     ((a) - (b))
#define FIXED_NEG(x)        (-(x))
#define FIXED_ABS(x)        ((x) >= 0 ? (x) : -(x))

// Multiplication et division 64-bit sûres
#define FIXED_MUL_64(a, b)  ((Fixed32)(((Fixed64)(a) * (Fixed64)(b)) >> FIXED_SHIFT))
#define FIXED_DIV_64(a, b)  ((Fixed32)(((Fixed64)(a) << FIXED_SHIFT) / (Fixed64)(b)))

// Constantes écran
#define CENTRE_X 160
#define CENTRE_Y 100

#define SCREEN_MODE 320
#define MAX_FACE_VERTICES 10

// ============================================================================
//  TABLE DE CONVERSION DEGRES -> RADIANS
// ============================================================================

static const Fixed32 deg_to_rad_table[361] = {
    0,      1143,   2287,   3430,   4573,   5717,   6860,   8003,   9147,   10290,
    11433,  12577,  13720,  14863,  16007,  17150,  18293,  19437,  20580,  21723,
    22867,  24010,  25153,  26297,  27440,  28583,  29727,  30870,  32013,  33157,
    34300,  35443,  36587,  37730,  38873,  40017,  41160,  42303,  43447,  44590,
    45733,  46877,  48020,  49163,  50307,  51450,  52593,  53737,  54880,  56023,
    57167,  58310,  59453,  60597,  61740,  62883,  64027,  65170,  66313,  67457,
    68600,  69743,  70887,  72030,  73173,  74317,  75460,  76603,  77747,  78890,
    80033,  81177,  82320,  83463,  84607,  85750,  86893,  88037,  89180,  90323,
    91467,  92610,  93753,  94897,  96040,  97183,  98327,  99470, 100613, 101757,
    102900,104043, 105187, 106330, 107473, 108617, 109760, 110903, 112047, 113190,
    114333,115477, 116620, 117763, 118907, 120050, 121193, 122337, 123480, 124623,
    125767,126910, 128053, 129197, 130340, 131483, 132627, 133770, 134913, 136057,
    137200,138343, 139487, 140630, 141773, 142917, 144060, 145203, 146347, 147490,
    148633,149777, 150920, 152063, 153207, 154350, 155493, 156637, 157780, 158923,
    160067,161210, 162353, 163497, 164640, 165783, 166927, 168070, 169213, 170357,
    171500,172643, 173787, 174930, 176073, 177217, 178360, 179503, 180647, 181790,
    182933,184077, 185220, 186363, 187507, 188650, 189793, 190937, 192080, 193223,
    194367,195510, 196653, 197797, 198940, 200083, 201227, 202370, 203513, 204657,
    205800,206943, 208087, 209230, 210373, 211517, 212660, 213803, 214947, 216090,
    217233,218377, 219520, 220663, 221807, 222950, 224093, 225237, 226380, 227523,
    228667,229810, 230953, 232097, 233240, 234383, 235527, 236670, 237813, 238957,
    240100,241243, 242387, 243530, 244673, 245817, 246960, 248103, 249247, 250390,
    251533,252677, 253820, 254963, 256107, 257250, 258393, 259537, 260680, 261823,
    262967,264110, 265253, 266397, 267540, 268683, 269827, 270970, 272113, 273257,
    274400,275543, 276687, 277830, 278973, 280117, 281260, 282403, 283547, 284690,
    285833,286977, 288120, 289263, 290407, 291550, 292693, 293837, 294980, 296123,
    297267,298410, 299553, 300697, 301840, 302983, 304127, 305270, 306413, 307557,
    308700,309843, 310987, 312130, 313273, 314417, 315560, 316703, 317847, 318990,
    320133,321277, 322420, 323563, 324707, 325850, 326993, 328137, 329280, 330423,
    331567,332710, 333853, 334997, 336140, 337283, 338427, 339570, 340713, 341857,
    343000,344143, 345287, 346430, 347573, 348717, 349860, 351003, 352147, 353290,
    354433,355577, 356720, 357863, 359007, 360150, 361293, 362437, 363580, 364723,
    365867,367010, 368153, 369297, 370440, 371583, 372727, 373870, 375013, 376157,
    377300,378443, 379587, 380730, 381873, 383017, 384160, 385303, 386447, 387590,
    388733,389877, 391020, 392163, 393307, 394450, 395593, 396737, 397880, 399023,
    400167,401310, 402453, 403597, 404740, 405883, 407027, 408170, 409313, 410457,
    411600
};

// ============================================================================
//  STRUCTURES
// ============================================================================

typedef struct {
    Fixed32 *x, *y, *z;      // Coordonnées 3D
    Fixed32 *zo;             // Profondeur transformée
    int *x2d, *y2d;          // Proj. 2D (coordonnées écran en int)
    int vertex_count;
} VertexArrays3D;

typedef struct {
    int *vertex_count;           // Nombre de sommets par face
    int *vertex_indices_ptr;     // Début dans le buffer pour chaque face
    int *vertex_indices_buffer;  // Tous les indices de toutes les faces
    int face_count;
    int total_indices;
} FaceArrays3D;

typedef struct {
    VertexArrays3D vertices;
    FaceArrays3D faces;
} Model3D;

typedef struct {
    Fixed32 distance;
    Fixed32 angle_h;
    Fixed32 angle_v;
    Fixed32 angle_w;
} ObserverParams;

// BSP Node (format binaire)
typedef struct {
    Word plane_face_idx;
    Word faces_on_plane_count;
    Word faces_on_plane_idx_start;
    int front_node_idx;
    int back_node_idx;
} BSPNode;

// Polygon dynamique compatible QuickDraw
typedef struct {
    int polySize;                          // Taille totale en octets
    Rect polyBBox;                         // Bounding box (type système)
    Point polyPoints[MAX_FACE_VERTICES];   // Points du polygone (type système)
} DynamicPolygon;

// Vertex3D pour lecture fichier
typedef struct {
    float x, y, z;
} Vertex3D;

// ============================================================================
//  VARIABLES GLOBALES
// ============================================================================

void* globalPolyHandle = 0L;
int poly_handle_locked = 0;
int max_polySize = 256;

// BSP data
BSPNode *bsp_nodes = NULL;
Word *bsp_faces_on_plane = NULL;
int bsp_node_count = 0;
int bsp_faces_on_plane_count = 0;

// ============================================================================
//  PROTOTYPES
// ============================================================================

Fixed32 sin_fixed(Fixed32 angle);
Fixed32 cos_fixed(Fixed32 angle);
void getObserverParams(ObserverParams* params);
void processModelFast(Model3D* model, ObserverParams* params);
int loadModelBSP(const char* filename, VertexArrays3D* vtx, FaceArrays3D* faces);
void setObserverPosition(ObserverParams* params);
void traverseAndDrawBSP(int node_idx, Model3D* model, VertexArrays3D* vtx, FaceArrays3D* faces, int vertex_count_total);
void printBSP(int node_idx, int depth);
void DoColor(void);
void DoText(void);
int main_bsp(int argc, char** argv);

// ============================================================================
//  FONCTIONS TRIGONOMETRIQUES EN VIRGULE FIXE
// ============================================================================

Fixed32 sin_fixed(Fixed32 angle) {
    while (angle > FIXED_PI) angle = FIXED_SUB(angle, FIXED_2PI);
    while (angle < -FIXED_PI) angle = FIXED_ADD(angle, FIXED_2PI);
    Fixed32 x = angle;
    Fixed32 x2 = FIXED_MUL_64(x, x);
    Fixed32 x3 = FIXED_MUL_64(x2, x);
    Fixed32 x5 = FIXED_MUL_64(x3, x2);
    Fixed32 x7 = FIXED_MUL_64(x5, x2);
    Fixed32 result = x;
    result = FIXED_SUB(result, FIXED_DIV_64(x3, INT_TO_FIXED(6)));
    result = FIXED_ADD(result, FIXED_DIV_64(x5, INT_TO_FIXED(120)));
    result = FIXED_SUB(result, FIXED_DIV_64(x7, INT_TO_FIXED(5040)));
    return result;
}

Fixed32 cos_fixed(Fixed32 angle) {
    return sin_fixed(FIXED_ADD(angle, FIXED_PI_2));
}

// ============================================================================
//  FONCTIONS PARAMÈTRES OBSERVATEUR ET TRANSFORMATION
// ============================================================================

void getObserverParams(ObserverParams* params) {
    char input[50];
    printf("\nObserver parameters:\n");
    printf("============================\n");
    printf("(Press ENTER to use default values)\n");
    
    printf("Horizontal angle (degrees, default 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_h = FLOAT_TO_FIXED(30.0);
        } else {
            params->angle_h = FLOAT_TO_FIXED(atof(input));
        }
    } else {
        params->angle_h = FLOAT_TO_FIXED(30.0);
    }
    
    printf("Vertical angle (degrees, default 20): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_v = FLOAT_TO_FIXED(20.0);
        } else {
            params->angle_v = FLOAT_TO_FIXED(atof(input));
        }
    } else {
        params->angle_v = FLOAT_TO_FIXED(20.0);
    }
    
    printf("Screen rotation angle (degrees, default 0): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_w = FLOAT_TO_FIXED(0.0);
        } else {
            params->angle_w = FLOAT_TO_FIXED(atof(input));
        }
    } else {
        params->angle_w = FLOAT_TO_FIXED(0.0);
    }
    
    printf("Distance (default 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->distance = FLOAT_TO_FIXED(30.0);
        } else {
            params->distance = FLOAT_TO_FIXED(atof(input));
        }
    } else {
        params->distance = FLOAT_TO_FIXED(30.0);
    }
}

void processModelFast(Model3D* model, ObserverParams* params) {
    int i;
    Fixed32 rad_h, rad_v, rad_w;
    Fixed32 cos_h, sin_h, cos_v, sin_v, cos_w, sin_w;
    Fixed32 x, y, z, zo, xo, yo;
    Fixed32 inv_zo, x2d_temp, y2d_temp;
    
    int angle_h_deg = FIXED_TO_INT(params->angle_h);
    int angle_v_deg = FIXED_TO_INT(params->angle_v);
    int angle_w_deg = FIXED_TO_INT(params->angle_w);
    if (angle_h_deg < 0) angle_h_deg = 0; if (angle_h_deg > 360) angle_h_deg = 360;
    if (angle_v_deg < 0) angle_v_deg = 0; if (angle_v_deg > 360) angle_v_deg = 360;
    if (angle_w_deg < 0) angle_w_deg = 0; if (angle_w_deg > 360) angle_w_deg = 360;
    
    rad_h = deg_to_rad_table[angle_h_deg];
    rad_v = deg_to_rad_table[angle_v_deg];
    rad_w = deg_to_rad_table[angle_w_deg];
    
    cos_h = cos_fixed(rad_h);
    sin_h = sin_fixed(rad_h);
    cos_v = cos_fixed(rad_v);
    sin_v = sin_fixed(rad_v);
    cos_w = cos_fixed(rad_w);
    sin_w = sin_fixed(rad_w);
    
    Fixed32 cos_h_cos_v = FIXED_MUL_64(cos_h, cos_v);
    Fixed32 sin_h_cos_v = FIXED_MUL_64(sin_h, cos_v);
    Fixed32 cos_h_sin_v = FIXED_MUL_64(cos_h, sin_v);
    Fixed32 sin_h_sin_v = FIXED_MUL_64(sin_h, sin_v);
    Fixed32 scale = FLOAT_TO_FIXED(100.0);
    Fixed32 centre_x_f = FLOAT_TO_FIXED((float)CENTRE_X);
    Fixed32 centre_y_f = FLOAT_TO_FIXED((float)CENTRE_Y);
    Fixed32 distance = params->distance;
    
    VertexArrays3D* vtx = &model->vertices;
    
    for (i = 0; i < vtx->vertex_count; i++) {
        x = vtx->x[i];
        y = vtx->y[i];
        z = vtx->z[i];
        Fixed32 term1 = FIXED_MUL_64(x, cos_h_cos_v);
        Fixed32 term2 = FIXED_MUL_64(y, sin_h_cos_v);
        Fixed32 term3 = FIXED_MUL_64(z, sin_v);
        zo = FIXED_ADD(FIXED_SUB(FIXED_SUB(FIXED_NEG(term1), term2), term3), distance);
        if (zo > 0) {
            xo = FIXED_ADD(FIXED_NEG(FIXED_MUL_64(x, sin_h)), FIXED_MUL_64(y, cos_h));
            yo = FIXED_ADD(FIXED_SUB(FIXED_NEG(FIXED_MUL_64(x, cos_h_sin_v)), FIXED_MUL_64(y, sin_h_sin_v)), FIXED_MUL_64(z, cos_v));
            vtx->zo[i] = zo;
            inv_zo = FIXED_DIV_64(scale, zo);
            x2d_temp = FIXED_ADD(FIXED_MUL_64(xo, inv_zo), centre_x_f);
            y2d_temp = FIXED_SUB(centre_y_f, FIXED_MUL_64(yo, inv_zo));
            vtx->x2d[i] = FIXED_TO_INT(FIXED_ADD(FIXED_SUB(FIXED_MUL_64(cos_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(sin_w, FIXED_SUB(centre_y_f, y2d_temp))), centre_x_f));
            vtx->y2d[i] = FIXED_TO_INT(FIXED_SUB(centre_y_f, FIXED_ADD(FIXED_MUL_64(sin_w, FIXED_SUB(x2d_temp, centre_x_f)), FIXED_MUL_64(cos_w, FIXED_SUB(centre_y_f, y2d_temp)))));
        } else {
            vtx->zo[i] = zo;
            vtx->x2d[i] = -1;
            vtx->y2d[i] = -1;
        }
    }
}

// ============================================================================
//  CHARGEMENT BSP
// ============================================================================

int loadModelBSP(const char* filename, VertexArrays3D* vtx, FaceArrays3D* faces) {
    printf("[DEBUG] Tentative d'ouverture du fichier BSP : %s\n", filename);
    FILE* f = fopen(filename, "rb");
    if (!f) {
        printf("[DEBUG] Erreur ouverture %s\n", filename);
        return -1;
    }
    Word vertex_count, face_count, node_count;
    fread(&vertex_count, 2, 1, f);
    fread(&face_count, 2, 1, f);
    fread(&node_count, 2, 1, f);
    printf("[DEBUG] BSP header: vertex_count=%u, face_count=%u, node_count=%u\n", vertex_count, face_count, node_count);
    keypress();

    // Allocation des tableaux de sommets
    vtx->x = (Fixed32*)malloc(vertex_count * sizeof(Fixed32));
    vtx->y = (Fixed32*)malloc(vertex_count * sizeof(Fixed32));
    vtx->z = (Fixed32*)malloc(vertex_count * sizeof(Fixed32));
    vtx->x2d = (int*)malloc(vertex_count * sizeof(int));
    vtx->y2d = (int*)malloc(vertex_count * sizeof(int));
    vtx->zo = (Fixed32*)malloc(vertex_count * sizeof(Fixed32));

    // Lecture des sommets
    for (int i = 0; i < vertex_count; i++) {
        float x, y, z;
        fread(&x, 4, 1, f);
        fread(&y, 4, 1, f);
        fread(&z, 4, 1, f);
        vtx->x[i] = (Fixed32)(x * FIXED_SCALE);
        vtx->y[i] = (Fixed32)(y * FIXED_SCALE);
        vtx->z[i] = (Fixed32)(z * FIXED_SCALE);
    }
    vtx->vertex_count = vertex_count;

    // Allocation des tableaux de faces (estimation)
    faces->vertex_count = (int*)malloc(face_count * sizeof(int));
    faces->vertex_indices_ptr = (int*)malloc(face_count * sizeof(int));
    faces->vertex_indices_buffer = (int*)malloc(face_count * 10 * sizeof(int)); // estimation max 10 vertices/face

    // Lecture des faces
    int idx = 0;
    long faces_bytes = 0;  // Taille totale des faces en octets
    for (int i = 0; i < face_count; i++) {
        Byte vpf;
        fread(&vpf, 1, 1, f);
        faces_bytes += 1;  // 1 octet pour vpf
        faces->vertex_count[i] = vpf;
        faces->vertex_indices_ptr[i] = idx;
        for (int j = 0; j < vpf; j++) {
            Word vidx;
            fread(&vidx, 2, 1, f);
            faces_bytes += 2;  // 2 octets par index
            faces->vertex_indices_buffer[idx++] = vidx;
        }
    }
    faces->face_count = face_count;
    faces->total_indices = idx;

    // Lecture des noeuds BSP
    bsp_nodes = (BSPNode*)malloc(sizeof(BSPNode) * node_count);
    for (int i = 0; i < node_count; i++) {
        fread(&bsp_nodes[i], sizeof(BSPNode), 1, f);
    }
    bsp_node_count = node_count;

    // Lecture faces_on_plane (concat de tous les noeuds)
    // Position = header(6) + vertices(vertex_count*12) + faces(faces_bytes) + nodes
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    long pos = 6 + vertex_count*12 + faces_bytes + node_count*sizeof(BSPNode);
    bsp_faces_on_plane_count = (file_size - pos) / 2;
    fseek(f, pos, SEEK_SET);
    bsp_faces_on_plane = (Word*)malloc(2 * bsp_faces_on_plane_count);
    fread(bsp_faces_on_plane, 2, bsp_faces_on_plane_count, f);
    fclose(f);
    return 0;
}

// ============================================================================
//  TRAVERSÉE ET DESSIN BSP
// ============================================================================

// Position de l'observateur en coordonnées monde (calculée depuis les params)
static Fixed32 obs_x, obs_y, obs_z;

void setObserverPosition(ObserverParams* params) {
    // La transformation dans processModelFast est :
    // zo = -x*cos_h*cos_v - y*sin_h*cos_v - z*sin_v + distance
    // xo = -x*sin_h + y*cos_h  
    // yo = -x*cos_h*sin_v - y*sin_h*sin_v + z*cos_v
    //
    // L'observateur est à (0,0,0) en espace vue, regardant vers +Z
    // En espace monde, l'observateur est à l'inverse de la transformation
    // appliquée à un point à distance "d" sur l'axe Z
    //
    // Position observateur en espace monde :
    // obs = R_inverse * (0, 0, distance)
    // où R est la matrice de rotation (h puis v)
    
    int ah = FIXED_TO_INT(params->angle_h);
    int av = FIXED_TO_INT(params->angle_v);
    if (ah < 0) ah += 360; if (ah >= 360) ah -= 360;
    if (av < 0) av += 360; if (av >= 360) av -= 360;
    
    Fixed32 rad_h = deg_to_rad_table[ah];
    Fixed32 rad_v = deg_to_rad_table[av];
    Fixed32 cos_h = cos_fixed(rad_h);
    Fixed32 sin_h = sin_fixed(rad_h);
    Fixed32 cos_v = cos_fixed(rad_v);
    Fixed32 sin_v = sin_fixed(rad_v);
    
    // L'observateur regarde depuis (distance * direction)
    // direction = (-cos_h*cos_v, -sin_h*cos_v, -sin_v) d'après la transfo
    // Donc position observateur = -distance * direction
    obs_x = FIXED_MUL_64(params->distance, FIXED_MUL_64(cos_h, cos_v));
    obs_y = FIXED_MUL_64(params->distance, FIXED_MUL_64(sin_h, cos_v));
    obs_z = FIXED_MUL_64(params->distance, sin_v);
}

// Calcule de quel côté du plan (défini par 3 vertices) se trouve l'observateur
// Retourne > 0 si devant (côté normal), < 0 si derrière, 0 si sur le plan
Fixed32 classifyPoint(int face_idx, FaceArrays3D* faces, VertexArrays3D* vtx) {
    if (face_idx < 0 || face_idx >= faces->face_count) return 0;
    if (faces->vertex_count[face_idx] < 3) return 0;
    
    int offset = faces->vertex_indices_ptr[face_idx];
    int v0 = faces->vertex_indices_buffer[offset];
    int v1 = faces->vertex_indices_buffer[offset + 1];
    int v2 = faces->vertex_indices_buffer[offset + 2];
    
    if (v0 < 0 || v0 >= vtx->vertex_count) return 0;
    if (v1 < 0 || v1 >= vtx->vertex_count) return 0;
    if (v2 < 0 || v2 >= vtx->vertex_count) return 0;
    
    // Vecteurs du plan : AB et AC
    Fixed32 abx = FIXED_SUB(vtx->x[v1], vtx->x[v0]);
    Fixed32 aby = FIXED_SUB(vtx->y[v1], vtx->y[v0]);
    Fixed32 abz = FIXED_SUB(vtx->z[v1], vtx->z[v0]);
    Fixed32 acx = FIXED_SUB(vtx->x[v2], vtx->x[v0]);
    Fixed32 acy = FIXED_SUB(vtx->y[v2], vtx->y[v0]);
    Fixed32 acz = FIXED_SUB(vtx->z[v2], vtx->z[v0]);
    
    // Normale = AB x AC
    Fixed32 nx = FIXED_SUB(FIXED_MUL_64(aby, acz), FIXED_MUL_64(abz, acy));
    Fixed32 ny = FIXED_SUB(FIXED_MUL_64(abz, acx), FIXED_MUL_64(abx, acz));
    Fixed32 nz = FIXED_SUB(FIXED_MUL_64(abx, acy), FIXED_MUL_64(aby, acx));
    
    // Vecteur du point A vers l'observateur
    Fixed32 apx = FIXED_SUB(obs_x, vtx->x[v0]);
    Fixed32 apy = FIXED_SUB(obs_y, vtx->y[v0]);
    Fixed32 apz = FIXED_SUB(obs_z, vtx->z[v0]);
    
    // Produit scalaire N . AP = distance signée
    Fixed32 dot = FIXED_ADD(FIXED_ADD(FIXED_MUL_64(nx, apx), FIXED_MUL_64(ny, apy)), FIXED_MUL_64(nz, apz));
    
    return dot;
}

void traverseAndDrawBSP(int node_idx, Model3D* model, VertexArrays3D* vtx, FaceArrays3D* faces, int vertex_count_total) {
    if (node_idx < 0 || node_idx >= bsp_node_count) return;
    BSPNode* node = &bsp_nodes[node_idx];

    // Déterminer de quel côté du plan se trouve l'observateur
    int plane_face = node->plane_face_idx;
    Fixed32 side = classifyPoint(plane_face, faces, vtx);
    
    // Si observateur devant le plan (side > 0): dessiner back, plan, front
    // Si observateur derrière le plan (side < 0): dessiner front, plan, back
    if (side > 0) {
        // Observateur devant : d'abord back (loin), puis plan, puis front (proche)
        if (node->back_node_idx >= 0)
            traverseAndDrawBSP(node->back_node_idx, model, vtx, faces, vertex_count_total);
    } else {
        // Observateur derrière : d'abord front, puis plan, puis back
        if (node->front_node_idx >= 0)
            traverseAndDrawBSP(node->front_node_idx, model, vtx, faces, vertex_count_total);
    }

    // Draw all faces on this plane
    for (int i = 0; i < node->faces_on_plane_count; i++) {
        int face_id = bsp_faces_on_plane[node->faces_on_plane_idx_start + i];
        // Only draw valid faces (3+ vertices)
        if (face_id >= 0 && face_id < faces->face_count && faces->vertex_count[face_id] >= 3) {
            Handle polyHandle;
            DynamicPolygon *poly;
            int min_x, max_x, min_y, max_y;
            Pattern pat;
            if (globalPolyHandle == NULL) {
                int max_polySize = 2 + 8 + (MAX_FACE_VERTICES * 4);
                globalPolyHandle = NewHandle((long)max_polySize, userid(), 0xC014, 0L);
                if (globalPolyHandle == NULL) {
                    printf("Error: Unable to allocate global polygon handle\n");
                    return;
                }
            }
            polyHandle = globalPolyHandle;
            if (poly_handle_locked) {
                HUnlock(polyHandle);
                poly_handle_locked = 0;
            }
            HLock(polyHandle);
            poly_handle_locked = 1;
            int offset = faces->vertex_indices_ptr[face_id];
            int vcount = faces->vertex_count[face_id];
            if (vcount > MAX_FACE_VERTICES) vcount = MAX_FACE_VERTICES;  // Limiter
            int polySize = 2 + 8 + (vcount * 4);
            poly = (DynamicPolygon *)*polyHandle;
            poly->polySize = polySize;
            min_x = max_x = min_y = max_y = -1;
            for (int j = 0; j < vcount; j++) {
                int vertex_idx = faces->vertex_indices_buffer[offset + j];  // Base 0, pas de -1
                if (vertex_idx >= 0 && vertex_idx < vtx->vertex_count) {
                    poly->polyPoints[j].h = SCREEN_MODE / 320 * vtx->x2d[vertex_idx];
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
            if (poly_handle_locked) {
                HUnlock(polyHandle);
                poly_handle_locked = 0;
            }
        }
    }

    // Dessiner l'autre sous-arbre
    if (side > 0) {
        // Observateur devant : finir par front (proche)
        if (node->front_node_idx >= 0)
            traverseAndDrawBSP(node->front_node_idx, model, vtx, faces, vertex_count_total);
    } else {
        // Observateur derrière : finir par back
        if (node->back_node_idx >= 0)
            traverseAndDrawBSP(node->back_node_idx, model, vtx, faces, vertex_count_total);
    }
}

void printBSP(int node_idx, int depth) {
    if (node_idx < 0 || node_idx >= bsp_node_count) return;
    for (int i = 0; i < depth; i++) printf("  ");
    printf("Node %d: plane_face=%d, faces_on_plane_count=%d, front=%d, back=%d\n", node_idx, bsp_nodes[node_idx].plane_face_idx, bsp_nodes[node_idx].faces_on_plane_count, bsp_nodes[node_idx].front_node_idx, bsp_nodes[node_idx].back_node_idx);
    // Affiche les faces sur ce plan
    for (int i = 0; i < bsp_nodes[node_idx].faces_on_plane_count; i++) {
        for (int j = 0; j < depth+1; j++) printf("  ");
        printf("face %d\n", bsp_faces_on_plane[bsp_nodes[node_idx].faces_on_plane_idx_start + i]);
    }
    if (bsp_nodes[node_idx].front_node_idx >= 0)
        printBSP(bsp_nodes[node_idx].front_node_idx, depth+1);
    if (bsp_nodes[node_idx].back_node_idx >= 0)
        printBSP(bsp_nodes[node_idx].back_node_idx, depth+1);
}

// ============================================================================
//  STUBS GRAPHIQUES
// ============================================================================

void DoColor(void) {
    // Stub : rien à faire
}

void DoText() {
        shroff();
        putchar((char) 12); // Clear screen    
}


// ============================================================================
//  MAIN BSP
// ============================================================================

int main_bsp(int argc, char** argv) {
    printf("\n==== GS3Dbsp - Chargement BSP binaire ====\n");
    Model3D model;
    ObserverParams params;
    char filename[100];
    int colorpalette = 0;

    memset(&model, 0, sizeof(Model3D));

    // Demande le nom du fichier BSP à l'utilisateur
    printf("Entrez le nom du fichier BSP à lire : ");
    if (fgets(filename, sizeof(filename), stdin) != NULL) {
        size_t len = strlen(filename);
        if (len > 0 && filename[len-1] == '\n') {
            filename[len-1] = '\0';
        }
    }

    // Charge le modèle BSP
    printf("nom du fichier: %s\n", filename);
    keypress();
    if (loadModelBSP(filename, &model.vertices, &model.faces) != 0) {
        printf("Erreur chargement BSP\n");
        return 1;
    }
    printf("\nBSP chargé: %d sommets, %d faces, %d noeuds\n", model.vertices.vertex_count, model.faces.face_count, bsp_node_count);

    // Get observer parameters
    getObserverParams(&params);

bigloop:
    printf("Processing model...\n");
    processModelFast(&model, &params);
    printf("Press any key to continue...\n");
    keypress();

loopReDraw:
    {
        int key = 0;
        if (model.faces.face_count > 0) {
            startgraph(SCREEN_MODE);
            SetPenMode(0);
            // Calculer la position de l'observateur pour la traversée BSP
            setObserverPosition(&params);
            // Draw using BSP traversal
            traverseAndDrawBSP(0, &model, &model.vertices, &model.faces, model.vertices.vertex_count);
            if (colorpalette == 1) { DoColor(); }

            asm {
                sep #0x20
            loop:
                lda >0xC000
                bpl loop
                and #0x007f
                sta >0xC010
                sta key
                rep #0x30
            }
            endgraph();
        }
        DoText();

        switch (key) {
            case 32:
                printf("===================================\n");
                printf(" Model information and parameters\n");
                printf("===================================\n");
                printf("Model: %s\n", filename);
                printf("Vertices: %d, Faces: %d\n", model.vertices.vertex_count, model.faces.face_count);
                printf("Observer Parameters:\n");
                printf("    Distance: %.2f\n", FIXED_TO_FLOAT(params.distance));
                printf("    Horizontal Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_h));
                printf("    Vertical Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_v));
                printf("    Screen Rotation Angle: %.1f\n", FIXED_TO_FLOAT(params.angle_w));
                printf("===================================\n\n");
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;
            case 65: case 97:
                params.distance = params.distance - (params.distance / 10);
                goto bigloop;
            case 90: case 122:
                params.distance = params.distance + (params.distance / 10);
                goto bigloop;
            case 21:
                params.angle_h = params.angle_h + INT_TO_FIXED(10);
                goto bigloop;
            case 8:
                params.angle_h = params.angle_h - INT_TO_FIXED(10);
                goto bigloop;
            case 10:
                params.angle_v = params.angle_v - INT_TO_FIXED(10);
                goto bigloop;
            case 11:
                params.angle_v = params.angle_v + INT_TO_FIXED(10);
                goto bigloop;
            case 87: case 119:
                params.angle_w = params.angle_w + INT_TO_FIXED(10);
                goto bigloop;
            case 88: case 120:
                params.angle_w = params.angle_w - INT_TO_FIXED(10);
                goto bigloop;
            case 67: case 99:
                colorpalette ^= 1;
                goto loopReDraw;
            case 78: case 110:
                // No reload in BSP mode
                goto loopReDraw;
            case 72: case 104:
                printf("===================================\n");
                printf("    HELP - Keyboard Controller\n");
                printf("===================================\n\n");
                printf("Space: Display model info\n");
                printf("A/Z: Increase/Decrease distance\n");
                printf("Arrow Left/Right: Decrease/Increase horizontal angle\n");
                printf("Arrow Up/Down: Increase/Decrease vertical angle\n");
                printf("W/X: Increase/Decrease screen rotation angle\n");
                printf("C: Toggle color palette display\n");
                printf("N: Load new model (not supported in BSP mode)\n");
                printf("H: Display this help message\n");
                printf("ESC: Quit program\n");
                printf("===================================\n\n");
                printf("Press any key to continue...\n");
                keypress();
                goto loopReDraw;
            case 27:
                goto end;
            default:
                goto loopReDraw;
        }
    }
end:
    if (globalPolyHandle != NULL) {
        if (poly_handle_locked) {
            HUnlock(globalPolyHandle);
        }
        DisposeHandle(globalPolyHandle);
        globalPolyHandle = NULL;
    }
    return 0;
}

// ============================================================================
//  POINT D'ENTREE PRINCIPAL
// ============================================================================

int main(int argc, char** argv) {
    return main_bsp(argc, argv);
}
