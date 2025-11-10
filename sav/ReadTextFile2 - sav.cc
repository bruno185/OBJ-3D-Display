#include <stdio.h>
#include <asm.h>
#include <string.h>
#include <misctool.h>
#include <stdlib.h>
#include <math.h>
#include <quickdraw.h>
#include <event.h>
#include <memory.h>
#include <window.h>
#include <orca.h>

#define MAX_LINE_LENGTH 256
#define MAX_VERTICES 1000
#define MAX_FACES 1000
#define MAX_FACE_VERTICES 20
#define PI 3.14159265359
#define CENTRE_X 160
#define CENTRE_Y 100

// Structure pour stocker un point 3D avec coordonnees originales et transformees
typedef struct {
    float x, y, z;          // Coordonnees originales du fichier
    float xo, yo, zo;       // Coordonnees transformees (observateur)
    int x2d, y2d;           // Coordonnees projetees sur l'ecran
} Vertex3D;

// Structure pour stocker une face (polygone)
typedef struct {
    int vertex_count;                           // Nombre de vertices dans la face
    int vertex_indices[MAX_FACE_VERTICES];     // Indices des vertices (base 1)
} Face3D;

// Structure pour un polygone QuickDraw dynamique
typedef struct {
    int polySize;
    Rect polyBBox;
    Point polyPoints[MAX_FACE_VERTICES];
} DynamicPolygon;

// Structure pour les parametres de l'observateur
typedef struct {
    float angle_h;      // Angle horizontal
    float angle_v;      // Angle vertical
    float distance;     // Distance
    float angle_w;      // Angle de rotation ecran
} ObserverParams;

// Structure pour contenir toutes les donnees du modele 3D
typedef struct {
    Vertex3D *vertices;
    Face3D *faces;
    int vertex_count;
    int face_count;
} Model3D;

// Declarations des fonctions
int readVertices(const char* filename, Vertex3D* vertices, int max_vertices);
int readFaces(const char* filename, Face3D* faces, int max_faces);
void transformToObserver(Vertex3D* vertices, int vertex_count, 
                        float angle_h, float angle_v, float distance);
void projectTo2D(Vertex3D* vertices, int vertex_count, float angle_w);
void drawPolygons(Vertex3D* vertices, Face3D* faces, int face_count);
void delay(int seconds);

// Fonctions pour la gestion du modele 3D
Model3D* createModel3D(void);
void destroyModel3D(Model3D* model);
int loadModel3D(Model3D* model, const char* filename);

// Fonctions pour l'interface utilisateur
void getObserverParams(ObserverParams* params);
void displayModelInfo(Model3D* model);
void displayResults(Model3D* model);
void processModel(Model3D* model, ObserverParams* params);

// *** IMPLEMENTATIONS ***

// Fonction pour attendre un nombre de secondes specifie
void delay(int seconds) {
    long startTick = GetTick();
    long ticksToWait = seconds * 60;  // 60 ticks par seconde
    while (GetTick() - startTick < ticksToWait) {
        // Attendre que le nombre de ticks requis se soit ecoule
    }
}

// *** FONCTIONS DE GESTION DU MODELE 3D ***

Model3D* createModel3D(void) {
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;
    }
    
    // Allouer la memoire pour les vertices
    model->vertices = (Vertex3D*)malloc(MAX_VERTICES * sizeof(Vertex3D));
    if (model->vertices == NULL) {
        free(model);
        return NULL;
    }
    
    // Allouer la memoire pour les faces
    model->faces = (Face3D*)malloc(MAX_FACES * sizeof(Face3D));
    if (model->faces == NULL) {
        free(model->vertices);
        free(model);
        return NULL;
    }
    
    model->vertex_count = 0;
    model->face_count = 0;
    
    return model;
}

void destroyModel3D(Model3D* model) {
    if (model != NULL) {
        if (model->vertices != NULL) {
            free(model->vertices);
        }
        if (model->faces != NULL) {
            free(model->faces);
        }
        free(model);
    }
}

int loadModel3D(Model3D* model, const char* filename) {
    if (model == NULL || filename == NULL) {
        return -1;
    }
    
    // Lire les vertices
    model->vertex_count = readVertices(filename, model->vertices, MAX_VERTICES);
    if (model->vertex_count < 0) {
        return -1;
    }
    
    // Lire les faces
    model->face_count = readFaces(filename, model->faces, MAX_FACES);
    if (model->face_count < 0) {
        printf("\nAvertissement: Impossible de lire les faces\n");
        model->face_count = 0;
    }
    
    return 0;
}

// *** FONCTIONS D'INTERFACE UTILISATEUR ***

void getObserverParams(ObserverParams* params) {
    char input[50];
    
    printf("\nParametres de l'observateur:\n");
    printf("============================\n");
    
    printf("Angle horizontal (degres): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        params->angle_h = atof(input);
    } else {
        params->angle_h = 0.0;
    }
    
    printf("Angle vertical (degres): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        params->angle_v = atof(input);
    } else {
        params->angle_v = 0.0;
    }
    
    printf("Distance: ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        params->distance = atof(input);
    } else {
        params->distance = 10.0;
    }
    
    printf("Angle de rotation ecran (degres): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        params->angle_w = atof(input);
    } else {
        params->angle_w = 0.0;
    }
}

void displayModelInfo(Model3D* model) {
    printf("\nResume de l'analyse:\n");
    printf("====================\n");
    printf("Nombre de vertices (points 3D) trouves: %d\n", model->vertex_count);
    printf("Nombre de faces trouvees: %d\n", model->face_count);
}

void displayResults(Model3D* model) {
    int i, j;
    
    if (model->vertex_count > 0) {
        printf("\nCoordonnees completes (Originales -> 3D -> 2D):\n");
        printf("-----------------------------------------------\n");
        for (i = 0; i < model->vertex_count; i++) {
            if (model->vertices[i].x2d >= 0 && model->vertices[i].y2d >= 0) {
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (%d,%d)\n", 
                       i + 1, 
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo,
                       model->vertices[i].x2d, model->vertices[i].y2d);
            } else {
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (invisible)\n", 
                       i + 1, 
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo);
            }
        }
    }
    
    if (model->face_count > 0) {
        printf("\nListe des faces:\n");
        printf("----------------\n");
        for (i = 0; i < model->face_count; i++) {
            printf("  Face %3d (%d vertices): ", i + 1, model->faces[i].vertex_count);
            for (j = 0; j < model->faces[i].vertex_count; j++) {
                printf("%d", model->faces[i].vertex_indices[j]);
                if (j < model->faces[i].vertex_count - 1) printf("-");
            }
            printf("\n");
        }
        
        // Dessiner les polygones avec QuickDraw
        drawPolygons(model->vertices, model->faces, model->face_count);
    }
}

void processModel(Model3D* model, ObserverParams* params) {
    // Appliquer la transformation des coordonnees
    transformToObserver(model->vertices, model->vertex_count, 
                       params->angle_h, params->angle_v, params->distance);
    
    // Projeter sur l'ecran 2D
    projectTo2D(model->vertices, model->vertex_count, params->angle_w);
}

// *** IMPLEMENTATIONS DES FONCTIONS DE BASE ***

// Fonction pour lire les vertices d'un fichier 3D
int readVertices(const char* filename, Vertex3D* vertices, int max_vertices) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    int line_number = 1;
    int vertex_count = 0;
    
    // Ouvrir le fichier en mode lecture
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Erreur: Impossible d'ouvrir le fichier '%s'\n", filename);
        printf("Verifiez que le fichier existe et que vous avez les permissions de lecture.\n");
        return -1;  // Retourner -1 en cas d'erreur
    }
    
    printf("\nContenu du fichier '%s':\n", filename);
    printf("========================\n\n");
    
    // Lire le fichier ligne par ligne
    while (fgets(line, sizeof(line), file) != NULL) {
        printf("%3d: %s", line_number, line);
        
        // Verifier si la ligne commence par "v " (vertex)
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;
                // Extraire les coordonnees x, y, z
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vertices[vertex_count].x = x;
                    vertices[vertex_count].y = y;
                    vertices[vertex_count].z = z;
                    vertex_count++;
                    printf("     -> Vertex %d: (%.3f, %.3f, %.3f)\n", 
                           vertex_count, x, y, z);
                }
            } else {
                printf("     -> ATTENTION: Limite de vertices atteinte (%d)\n", max_vertices);
            }
        }
        
        line_number++;
    }
    
    // Fermer le fichier
    fclose(file);
    
    printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
    return vertex_count;  // Retourner le nombre de vertices lus
}

// Fonction pour lire les faces d'un fichier 3D
int readFaces(const char* filename, Face3D* faces, int max_faces) {
    FILE *file;
    char line[MAX_LINE_LENGTH];
    char *token;
    int line_number = 1;
    int face_count = 0;
    int i;
    
    // Ouvrir le fichier en mode lecture
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Erreur: Impossible d'ouvrir le fichier '%s' pour lire les faces\n", filename);
        return -1;  // Retourner -1 en cas d'erreur
    }
    
    printf("\nLecture des faces du fichier '%s':\n", filename);
    printf("==================================\n\n");
    
    // Lire le fichier ligne par ligne
    while (fgets(line, sizeof(line), file) != NULL) {
        // Verifier si la ligne commence par "f " (face)
        if (line[0] == 'f' && line[1] == ' ') {
            if (face_count < max_faces) {
                printf("%3d: %s", line_number, line);
                
                faces[face_count].vertex_count = 0;
                
                // Utiliser strtok pour separer les indices des vertices
                token = strtok(line + 2, " \t\n");  // Commencer apres "f "
                while (token != NULL && faces[face_count].vertex_count < MAX_FACE_VERTICES) {
                    int vertex_index = atoi(token);
                    if (vertex_index > 0) {  // Les indices commencent a 1
                        faces[face_count].vertex_indices[faces[face_count].vertex_count] = vertex_index;
                        faces[face_count].vertex_count++;
                    }
                    token = strtok(NULL, " \t\n");
                }
                
                printf("     -> Face %d: %d vertices (", face_count + 1, faces[face_count].vertex_count);
                for (i = 0; i < faces[face_count].vertex_count; i++) {
                    printf("%d", faces[face_count].vertex_indices[i]);
                    if (i < faces[face_count].vertex_count - 1) printf(",");
                }
                printf(")\n");
                
                face_count++;
            } else {
                printf("     -> ATTENTION: Limite de faces atteinte (%d)\n", max_faces);
            }
        }
        
        line_number++;
    }
    
    // Fermer le fichier
    fclose(file);
    
    printf("\n\nAnalyse des faces terminee. %d faces lues.\n", face_count);
    return face_count;  // Retourner le nombre de faces lues
}

// Fonction pour transformer les coordonnees vers le systeme observateur
void transformToObserver(Vertex3D* vertices, int vertex_count, 
                        float angle_h, float angle_v, float distance) {
    int i;
    float rad_h, rad_v;
    float cos_h, sin_h, cos_v, sin_v;
    float x, y, z;
    
    // Convertir les angles en radians
    rad_h = angle_h * PI / 180.0;
    rad_v = angle_v * PI / 180.0;
    
    // Precalculer les valeurs trigonometriques
    cos_h = cos(rad_h);
    sin_h = sin(rad_h);
    cos_v = cos(rad_v);
    sin_v = sin(rad_v);
    
    printf("\nTransformation vers le systeme observateur:\n");
    printf("Angle horizontal: %.1f degres\n", angle_h);
    printf("Angle vertical: %.1f degres\n", angle_v);
    printf("Distance: %.3f\n", distance);
    printf("==========================================\n");
    
    // Transformer chaque vertex
    for (i = 0; i < vertex_count; i++) {
        // Utiliser les coordonnees originales (preservees)
        x = vertices[i].x;
        y = vertices[i].y;
        z = vertices[i].z;
        
        // Calculer les coordonnees transformees dans les nouveaux champs
        vertices[i].zo = -x * (cos_h * cos_v) - y * (sin_h * cos_v) - z * sin_v + distance;
        vertices[i].xo = -x * sin_h + y * cos_h;
        vertices[i].yo = -x * (cos_h * sin_v) - y * (sin_h * sin_v) + z * cos_v;
        
        printf("Vertex %3d: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", 
               i + 1, x, y, z, vertices[i].xo, vertices[i].yo, vertices[i].zo);
    }
}

// Fonction pour projeter les coordonnees 3D sur l'ecran 2D
void projectTo2D(Vertex3D* vertices, int vertex_count, float angle_w) {
    int i;
    float rad_w;
    float cos_w, sin_w;
    float x2d_temp, y2d_temp, save_x;
    
    // Convertir l'angle en radians
    rad_w = angle_w * PI / 180.0;
    
    // Precalculer les valeurs trigonometriques
    cos_w = cos(rad_w);
    sin_w = sin(rad_w);
    
    printf("\nProjection sur l'ecran 2D:\n");
    printf("Angle de rotation: %.1f degres\n", angle_w);
    printf("Centre ecran: (%d, %d)\n", CENTRE_X, CENTRE_Y);
    printf("===========================\n");
    
    // Projeter chaque vertex
    for (i = 0; i < vertex_count; i++) {
        // Verifier que le point est devant l'observateur (zo > 0)
        if (vertices[i].zo > 0) {
            // Projection perspective simple
            x2d_temp = (vertices[i].xo * 100.0) / vertices[i].zo + CENTRE_X;
            y2d_temp = CENTRE_Y - (vertices[i].yo * 100.0) / vertices[i].zo;
            
            // Sauvegarder x2d_temp pour la rotation
            save_x = x2d_temp;
            
            // Appliquer la rotation autour du centre de l'ecran
            vertices[i].x2d = (int)(cos_w * (x2d_temp - CENTRE_X) - sin_w * (CENTRE_Y - y2d_temp)) + CENTRE_X;
            vertices[i].y2d = CENTRE_Y - (int)(sin_w * (save_x - CENTRE_X) + cos_w * (CENTRE_Y - y2d_temp));
            
            printf("Vertex %3d: 3D(%.2f,%.2f,%.2f) -> 2D(%d,%d)\n", 
                   i + 1, vertices[i].xo, vertices[i].yo, vertices[i].zo, 
                   vertices[i].x2d, vertices[i].y2d);
        } else {
            // Point derriere l'observateur, pas de projection
            vertices[i].x2d = -1;
            vertices[i].y2d = -1;
            printf("Vertex %3d: Derriere l'observateur (zo=%.2f)\n", 
                   i + 1, vertices[i].zo);
        }
    }
}

// Fonction pour dessiner les polygones avec QuickDraw
void drawPolygons(Vertex3D* vertices, Face3D* faces, int face_count) {
    int i, j;
    Handle polyHandle;
    DynamicPolygon *poly;
    int visible_vertices;
    int min_x, max_x, min_y, max_y;
    
    printf("\nDessin des polygones avec QuickDraw:\n");
    printf("====================================\n");
    
    // Initialiser QuickDraw
    startgraph(320);
    
    // Dessiner chaque face
    for (i = 0; i < face_count; i++) {
        // Verifier que tous les vertices de la face sont visibles
        visible_vertices = 0;
        for (j = 0; j < faces[i].vertex_count; j++) {
            int vertex_idx = faces[i].vertex_indices[j] - 1; // Convertir base 1 vers base 0
            if (vertex_idx >= 0 && vertices[vertex_idx].x2d >= 0 && vertices[vertex_idx].y2d >= 0) {
                visible_vertices++;
            }
        }
        
        // Dessiner seulement si au moins 3 vertices sont visibles
        if (visible_vertices >= 3 && faces[i].vertex_count >= 3) {
            // Calculer la taille du polygone (en-tete + bbox + points)
            int polySize = 2 + 8 + (faces[i].vertex_count * 4);
            
            // Allouer la memoire pour le polygone
            polyHandle = NewHandle((long)polySize, userid(), 0xC015, 0L);
            if (polyHandle != NULL) {
                HLock(polyHandle);
                poly = (DynamicPolygon *)*polyHandle;
                
                // Remplir la structure du polygone
                poly->polySize = polySize;
                
                // Initialiser les bornes de la bounding box
                min_x = max_x = min_y = max_y = -1;
                
                // Copier les points et calculer la bounding box
                for (j = 0; j < faces[i].vertex_count; j++) {
                    int vertex_idx = faces[i].vertex_indices[j] - 1;
                    if (vertex_idx >= 0) {
                        poly->polyPoints[j].h = vertices[vertex_idx].x2d;
                        poly->polyPoints[j].v = vertices[vertex_idx].y2d;
                        
                        // Mettre a jour la bounding box
                        if (min_x == -1 || vertices[vertex_idx].x2d < min_x) min_x = vertices[vertex_idx].x2d;
                        if (max_x == -1 || vertices[vertex_idx].x2d > max_x) max_x = vertices[vertex_idx].x2d;
                        if (min_y == -1 || vertices[vertex_idx].y2d < min_y) min_y = vertices[vertex_idx].y2d;
                        if (max_y == -1 || vertices[vertex_idx].y2d > max_y) max_y = vertices[vertex_idx].y2d;
                    }
                }
                
                // Definir la bounding box
                poly->polyBBox.h1 = min_x;
                poly->polyBBox.v1 = min_y;
                poly->polyBBox.h2 = max_x;
                poly->polyBBox.v2 = max_y;
                
                // Definir la couleur (cyclique pour varier les couleurs)
                SetSolidPenPat((i % 15) + 1);
                
                // Dessiner le polygone
                PaintPoly(polyHandle);
                
                // Nettoyer
                HUnlock(polyHandle);
                DisposeHandle(polyHandle);
                
                printf("Face %d dessinee (%d vertices)\n", i + 1, faces[i].vertex_count);
            } else {
                printf("Erreur: Impossible d'allouer la memoire pour la face %d\n", i + 1);
            }
        } else {
            printf("Face %d ignoree (vertices invisibles: %d/%d)\n", 
                   i + 1, visible_vertices, faces[i].vertex_count);
        }
    }
    
    printf("\nDessin termine. Appuyez sur une touche pour continuer...\n");
    keypress();
}

// ****************************************************************************
//                              Fonction main
// ****************************************************************************

int main() {
    Model3D* model;
    ObserverParams params;
    char filename[100];
    char input[50];
    
    printf("Lecture de fichier 3D\n");
    printf("===================================\n\n");
    
    // Creer le modele 3D
    model = createModel3D();
    if (model == NULL) {
        printf("Erreur: Impossible d'allouer la memoire pour le modele 3D\n");
        printf("Press any key to quit...\n");
        keypress();
        return 1;
    }
    
    // Demander le nom du fichier
    printf("Entrez le nom du fichier a lire: ");
    if (fgets(filename, sizeof(filename), stdin) != NULL) {
        size_t len = strlen(filename);
        if (len > 0 && filename[len-1] == '\n') {
            filename[len-1] = '\0';
        }
    }
    
    // Charger le modele 3D
    if (loadModel3D(model, filename) < 0) {
        printf("\nErreur lors du chargement du fichier\n");
        printf("Press any key to quit...\n");
        keypress();
        destroyModel3D(model);
        return 1;
    }
    
    // Obtenir les parametres de l'observateur
    getObserverParams(&params);
    
    // Traiter le modele avec les parametres
    processModel(model, &params);
    
    // Afficher les informations et resultats
    displayModelInfo(model);
    displayResults(model);
    
    // Boucle interactive pour recalcul
    while (1) {
        printf("\n\nVoulez-vous recalculer avec de nouveaux angles? (o/n): ");
        if (fgets(input, sizeof(input), stdin) != NULL) {
            if (input[0] == 'n' || input[0] == 'N') {
                break;
            } else if (input[0] == 'o' || input[0] == 'O') {
                printf("\nNouveaux parametres de l'observateur:\n");
                printf("=====================================\n");
                getObserverParams(&params);
                processModel(model, &params);
                
                if (model->face_count > 0) {
                    drawPolygons(model->vertices, model->faces, model->face_count);
                }
            }
        }
    }
    
    // Nettoyage et fin
    printf("\nPress any key to quit...\n");
    debug();
    keypress();
    printf("\f");
    printf("Goodbye!\n");
    delay(1);
    
    destroyModel3D(model);
    return 0;
}