/*
 * ============================================================================
 *                              GO3D.CC
 * ============================================================================
 * 
 * Programme de rendu 3D pour Apple IIGS avec ORCA/C
 * 
 * DESCRIPTION:
 *   Ce programme lit des fichiers de modeles 3D au format OBJ simplifie,
 *   applique des transformations geometriques 3D (rotation, translation),
 *   projette les points sur un ecran 2D, et dessine les polygones resultants
 *   en utilisant QuickDraw.
 * 
 * FONCTIONNALITES:
 *   - Lecture de fichiers OBJ (vertices "v" et faces "f")
 *   - Transformations 3D vers systeme observateur
 *   - Projection perspective sur ecran 2D
 *   - Rendu graphique avec QuickDraw
 *   - Interface interactive pour modifier les parametres
 * 
 * AUTEUR: Bruno
 * DATE: 2025
 * PLATEFORME: Apple IIGS - ORCA/C
 * ============================================================================
 */

// ============================================================================
//                           INCLUSIONS D'EN-TETES
// ============================================================================

#include <stdio.h>      // Entrees/sorties standard (printf, fgets, etc.)
#include <asm.h>        // Fonctions assembleur specifiques ORCA
#include <string.h>     // Manipulation de chaines (strlen, strcmp, etc.)
#include <misctool.h>   // Outils divers ORCA (GetTick, keypress, etc.)
#include <stdlib.h>     // Fonctions standard (malloc, free, atof, etc.)
#include <math.h>       // Fonctions mathematiques (cos, sin, sqrt, etc.)
#include <quickdraw.h>  // API graphique QuickDraw d'Apple IIGS
#include <event.h>      // Gestion des evenements systeme
#include <memory.h>     // Gestion memoire avancee (NewHandle, etc.)
#include <window.h>     // Gestion des fenetres
#include <orca.h>       // Fonctions specifiques ORCA (startgraph, etc.)

// ============================================================================
//                            CONSTANTES GLOBALES
// ============================================================================

#define MAX_LINE_LENGTH 256     // Taille maximale d'une ligne de fichier
#define MAX_VERTICES 1000       // Nombre maximum de vertices dans un modele 3D
#define MAX_FACES 1000          // Nombre maximum de faces dans un modele 3D
#define MAX_FACE_VERTICES 20    // Nombre maximum de vertices par face (polygone)
#define PI 3.14159265359        // Constante mathematique Pi
#define CENTRE_X 160            // Centre de l'ecran en X (320/2)
#define CENTRE_Y 100            // Centre de l'ecran en Y (200/2)
//#define mode 640               // Mode graphique 640x200 pixels
#define mode 320               // Mode graphique 320x200 pixels

// ============================================================================
//                          STRUCTURES DE DONNEES
// ============================================================================

/**
 * Structure Vertex3D
 * 
 * DESCRIPTION:
 *   Represente un point dans l'espace 3D avec ses differentes representations
 *   au cours du pipeline de rendu 3D.
 * 
 * CHAMPS:
 *   x, y, z    : Coordonnees originales lues depuis le fichier OBJ
 *   xo, yo, zo : Coordonnees transformees dans le systeme observateur
 *                (apres application des rotations et translation)
 *   x2d, y2d   : Coordonnees finales projetees sur l'ecran 2D
 * 
 * UTILISATION:
 *   Cette structure preserve toutes les etapes de transformation pour
 *   permettre le debug et les recalculs sans relecture du fichier.
 */
typedef struct {
    float x, y, z;          // Coordonnees originales du fichier OBJ
    float xo, yo, zo;       // Coordonnees transformees (systeme observateur)
    int x2d, y2d;           // Coordonnees projetees sur l'ecran 2D (pixels)
} Vertex3D;

/**
 * Structure Face3D
 * 
 * DESCRIPTION:
 *   Represente une face (polygone) d'un objet 3D. Une face est definie
 *   par une liste d'indices pointant vers des vertices dans le tableau
 *   de vertices du modele.
 * 
 * CHAMPS:
 *   vertex_count    : Nombre de vertices composant cette face (3+ pour polygone)
 *   vertex_indices  : Tableau des indices des vertices (numerotation base 1
 *                     comme dans le format OBJ standard)
 * 
 * NOTES:
 *   - Les indices sont stockes en base 1 (premier vertex = indice 1)
 *   - Conversion en base 0 necessaire pour acceder au tableau C
 *   - Maximum MAX_FACE_VERTICES vertices par face pour eviter les debordements
 */
typedef struct {
    int vertex_count;                           // Nombre de vertices dans la face
    int vertex_indices[MAX_FACE_VERTICES];     // Indices des vertices (base 1)
    float z_max;                               // Profondeur maximale de la face (pour tri)
} Face3D;

/**
 * Structure DynamicPolygon
 * 
 * DESCRIPTION:
 *   Structure compatible avec QuickDraw pour dessiner des polygones.
 *   Cette structure doit etre allouee dynamiquement car sa taille
 *   varie selon le nombre de points du polygone.
 * 
 * CHAMPS:
 *   polySize    : Taille totale de la structure en octets
 *   polyBBox    : Rectangle englobant le polygone (bounding box)
 *   polyPoints  : Tableau des points du polygone en coordonnees ecran
 * 
 * FORMAT QUICKDRAW:
 *   QuickDraw attend une structure avec en-tete (taille + bbox) suivi
 *   des points. La taille doit inclure l'en-tete + tous les points.
 */
typedef struct {
    int polySize;                               // Taille totale structure (octets)
    Rect polyBBox;                             // Rectangle englobant (bounding box)
    Point polyPoints[MAX_FACE_VERTICES];       // Points du polygone (coordonnees ecran)
} DynamicPolygon;

/**
 * Structure ObserverParams
 * 
 * DESCRIPTION:
 *   Contient tous les parametres definissant la position et orientation
 *   de l'observateur (camera) dans l'espace 3D, ainsi que les parametres
 *   de projection.
 * 
 * CHAMPS:
 *   angle_h  : Angle horizontal de rotation de l'observateur (degres)
 *              Rotation autour de l'axe Y (gauche/droite)
 *   angle_v  : Angle vertical de rotation de l'observateur (degres)
 *              Rotation autour de l'axe X (haut/bas)
 *   angle_w  : Angle de rotation de la projection ecran (degres)
 *              Rotation dans le plan 2D final
 *   distance : Distance de l'observateur au centre du modele
 *              Plus grande = objet plus petit, plus petite = objet plus grand
 * 
 * NOTES MATHEMATIQUES:
 *   - Les angles sont en degres (convertis en radians pour les calculs)
 *   - La distance affecte la perspective et la taille apparente
 *   - angle_w permet une rotation finale pour ajuster l'orientation
 */
typedef struct {
    float angle_h;      // Angle horizontal de l'observateur (rotation Y)
    float angle_v;      // Angle vertical de l'observateur (rotation X)
    float angle_w;      // Angle de rotation de la projection 2D
    float distance;     // Distance observateur-objet (perspective)
} ObserverParams;

/**
 * Structure Model3D
 * 
 * DESCRIPTION:
 *   Structure principale contenant toutes les donnees d'un modele 3D.
 *   Elle regroupe les vertices, les faces, et les compteurs associes.
 * 
 * CHAMPS:
 *   vertices      : Pointeur vers tableau dynamique de vertices
 *   faces         : Pointeur vers tableau dynamique de faces
 *   vertex_count  : Nombre reel de vertices charges
 *   face_count    : Nombre reel de faces chargees
 * 
 * GESTION MEMOIRE:
 *   - Les tableaux sont alloues dynamiquement (malloc)
 *   - Permet de depasser les limites de la pile Apple IIGS
 *   - Liberation obligatoire avec destroyModel3D()
 * 
 * UTILISATION:
 *   Model3D* model = createModel3D();
 *   loadModel3D(model, "fichier.obj");
 *   // ... utilisation ...
 *   destroyModel3D(model);
 */
typedef struct {
    Vertex3D *vertices;     // Tableau dynamique des vertices du modele
    Face3D *faces;          // Tableau dynamique des faces du modele
    int vertex_count;       // Nombre reel de vertices charges
    int face_count;         // Nombre reel de faces chargees
} Model3D;

// ============================================================================
//                       DECLARATIONS DES FONCTIONS
// ============================================================================

/**
 * FONCTIONS DE LECTURE DE FICHIERS OBJ
 * ====================================
 */

/**
 * readVertices
 * 
 * DESCRIPTION:
 *   Lit les vertices (points 3D) depuis un fichier au format OBJ.
 *   Recherche les lignes commencant par "v " et extrait les coordonnees X,Y,Z.
 * 
 * PARAMETRES:
 *   filename     : Nom du fichier OBJ a lire
 *   vertices     : Tableau de destination pour stocker les vertices
 *   max_vertices : Taille maximale du tableau (protection debordement)
 * 
 * RETOUR:
 *   Nombre de vertices lus avec succes, ou -1 en cas d'erreur
 * 
 * FORMAT OBJ:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14
 */
int readVertices(const char* filename, Vertex3D* vertices, int max_vertices);

/**
 * readFaces
 * 
 * DESCRIPTION:
 *   Lit les faces (polygones) depuis un fichier au format OBJ.
 *   Recherche les lignes commencant par "f " et extrait les indices des vertices.
 * 
 * PARAMETRES:
 *   filename   : Nom du fichier OBJ a lire
 *   faces      : Tableau de destination pour stocker les faces
 *   max_faces  : Taille maximale du tableau (protection debordement)
 * 
 * RETOUR:
 *   Nombre de faces lues avec succes, ou -1 en cas d'erreur
 * 
 * FORMAT OBJ:
 *   f 1 2 3        (triangle avec vertices 1, 2, 3)
 *   f 4 5 6 7      (quadrilatere avec vertices 4, 5, 6, 7)
 */
int readFaces(const char* filename, Face3D* faces, int max_faces);

/**
 * FONCTIONS DE TRANSFORMATION GEOMETRIQUE 3D
 * ==========================================
 */

/**
 * transformToObserver
 * 
 * DESCRIPTION:
 *   Applique les transformations geometriques 3D pour passer du systeme
 *   de coordonnees du modele au systeme de coordonnees de l'observateur.
 *   
 *   TRANSFORMATIONS APPLIQUEES:
 *   1. Rotation horizontale (angle_h) autour de l'axe Y
 *   2. Rotation verticale (angle_v) autour de l'axe X
 *   3. Translation par la distance d'observation
 * 
 * PARAMETRES:
 *   vertices     : Tableau des vertices a transformer
 *   vertex_count : Nombre de vertices dans le tableau
 *   angle_h      : Angle de rotation horizontale (degres)
 *   angle_v      : Angle de rotation verticale (degres)
 *   distance     : Distance d'observation (translation en Z)
 * 
 * FORMULES MATHEMATIQUES:
 *   zo = -x*(cos_h*cos_v) - y*(sin_h*cos_v) - z*sin_v + distance
 *   xo = -x*sin_h + y*cos_h
 *   yo = -x*(cos_h*sin_v) - y*(sin_h*sin_v) + z*cos_v
 * 
 * COORDONNEES RESULTANTES:
 *   Les champs xo, yo, zo des vertices sont mis a jour.
 */
void transformToObserver(Vertex3D* vertices, int vertex_count, 
                        float angle_h, float angle_v, float distance);

/**
 * projectTo2D
 * 
 * DESCRIPTION:
 *   Projette les coordonnees 3D transformees sur un ecran 2D en utilisant
 *   une projection perspective. Applique egalement une rotation finale
 *   dans le plan 2D.
 * 
 * PARAMETRES:
 *   vertices     : Tableau des vertices a projeter
 *   vertex_count : Nombre de vertices dans le tableau
 *   angle_w      : Angle de rotation dans le plan 2D (degres)
 * 
 * ALGORITHME:
 *   1. Projection perspective: x2d = (xo * echelle) / zo + centre_x
 *   2. Idem pour y2d avec inversion de l'axe Y
 *   3. Rotation finale dans le plan 2D selon angle_w
 *   4. Points derriere l'observateur (zo <= 0) marques invisibles
 * 
 * COORDONNEES RESULTANTES:
 *   Les champs x2d, y2d des vertices contiennent les coordonnees ecran finales.
 */
void projectTo2D(Vertex3D* vertices, int vertex_count, float angle_w);

/**
 * FONCTIONS DE RENDU GRAPHIQUE
 * ============================
 */

/**
 * drawPolygons
 * 
 * DESCRIPTION:
 *   Dessine tous les polygones (faces) du modele 3D a l'ecran en utilisant
 *   l'API QuickDraw d'Apple IIGS. Chaque face est rendue avec une couleur
 *   differente pour la visualisation.
 * 
 * PARAMETRES:
 *   vertices   : Tableau des vertices avec coordonnees 2D calculees
 *   faces      : Tableau des faces a dessiner
 *   face_count : Nombre de faces dans le tableau
 * 
 * ALGORITHME:
 *   1. Initialisation du mode graphique QuickDraw
 *   2. Pour chaque face:
 *      - Verification visibilite des vertices
 *      - Creation structure polygon QuickDraw
 *      - Calcul bounding box
 *      - Allocation memoire dynamique
 *      - Dessin avec PaintPoly()
 *      - Liberation memoire
 * 
 * GESTION COULEURS:
 *   Couleurs cycliques basees sur l'index de la face (i % 15 + 1)
 * 
 * OPTIMISATIONS:
 *   - Faces avec moins de 3 vertices visibles ignorees
 *   - Vertices hors ecran geres correctement
 */
void drawPolygons(Vertex3D* vertices, Face3D* faces, int face_count, int vertex_count);
void calculateFaceDepths(Vertex3D* vertices, Face3D* faces, int face_count);
void sortFacesByDepth(Face3D* faces, int face_count);
void sortFacesByDepth_insertion(Face3D* faces, int face_count);
void sortFacesByDepth_quicksort(Face3D* faces, int low, int high);
void sortFacesByDepth_insertion_range(Face3D* faces, int low, int high);
int partition_median3(Face3D* faces, int low, int high);

// Fonction pour sauvegarder les donnees de debug sur disque
void saveDebugData(Model3D* model, const char* debug_filename);

/**
 * FONCTIONS UTILITAIRES
 * =====================
 */

/**
 * delay
 * 
 * DESCRIPTION:
 *   Attendre un nombre specifique de secondes en utilisant le timer
 *   systeme Apple IIGS.
 * 
 * PARAMETRES:
 *   seconds : Nombre de secondes a attendre
 * 
 * IMPLEMENTATION:
 *   Utilise GetTick() qui retourne le nombre de ticks depuis le demarrage
 *   (60 ticks par seconde sur Apple IIGS).
 */
void delay(int seconds);

/**
 * FONCTIONS DE GESTION DU MODELE 3D
 * =================================
 */

/**
 * createModel3D
 * 
 * DESCRIPTION:
 *   Cree et initialise une nouvelle structure Model3D avec allocation
 *   dynamique des tableaux de vertices et faces.
 * 
 * RETOUR:
 *   Pointeur vers la nouvelle structure, ou NULL en cas d'erreur memoire
 * 
 * GESTION MEMOIRE:
 *   - Allocation de la structure principale
 *   - Allocation du tableau de vertices (MAX_VERTICES)
 *   - Allocation du tableau de faces (MAX_FACES)
 *   - Cleanup automatique en cas d'echec partiel
 */
Model3D* createModel3D(void);

/**
 * destroyModel3D
 * 
 * DESCRIPTION:
 *   Libere toute la memoire associee a un modele 3D.
 * 
 * PARAMETRES:
 *   model : Pointeur vers le modele a detruire (peut etre NULL)
 * 
 * LIBERATION:
 *   - Tableau des vertices
 *   - Tableau des faces  
 *   - Structure principale
 */
void destroyModel3D(Model3D* model);

/**
 * loadModel3D
 * 
 * DESCRIPTION:
 *   Charge un modele 3D complet depuis un fichier OBJ.
 *   Combine la lecture des vertices et des faces.
 * 
 * PARAMETRES:
 *   model    : Structure Model3D de destination
 *   filename : Nom du fichier OBJ a charger
 * 
 * RETOUR:
 *   0 en cas de succes, -1 en cas d'erreur
 * 
 * TRAITEMENT:
 *   - Lecture des vertices avec readVertices()
 *   - Lecture des faces avec readFaces()
 *   - Mise a jour des compteurs dans la structure
 */
int loadModel3D(Model3D* model, const char* filename);

/**
 * FONCTIONS D'INTERFACE UTILISATEUR
 * =================================
 */

/**
 * getObserverParams
 * 
 * DESCRIPTION:
 *   Interface utilisateur pour saisir les parametres de l'observateur
 *   (angles, distance, etc.).
 * 
 * PARAMETRES:
 *   params : Structure de destination pour les parametres
 * 
 * INTERFACE:
 *   Demande interactive a l'utilisateur:
 *   - Angle horizontal
 *   - Angle vertical  
 *   - Distance d'observation
 *   - Angle de rotation ecran
 */
void getObserverParams(ObserverParams* params);

/**
 * displayModelInfo
 * 
 * DESCRIPTION:
 *   Affiche les informations generales sur le modele charge
 *   (nombre de vertices, nombre de faces).
 * 
 * PARAMETRES:
 *   model : Modele dont afficher les informations
 */
void displayModelInfo(Model3D* model);

/**
 * displayResults
 * 
 * DESCRIPTION:
 *   Affiche les resultats detailles du pipeline de rendu:
 *   coordonnees originales, transformees, et projetees.
 *   Lance egalement le rendu graphique.
 * 
 * PARAMETRES:
 *   model : Modele dont afficher les resultats
 */
void displayResults(Model3D* model);

/**
 * processModel
 * 
 * DESCRIPTION:
 *   Execute le pipeline complet de transformation 3D sur un modele:
 *   transformation vers observateur puis projection 2D.
 * 
 * PARAMETRES:
 *   model  : Modele a traiter
 *   params : Parametres de transformation
 */
void processModel(Model3D* model, ObserverParams* params);

// ============================================================================
//                          IMPLEMENTATIONS DES FONCTIONS
// ============================================================================

/**
 * FONCTION UTILITAIRE: delay
 * ==========================
 * 
 * Cette fonction utilise le timer systeme Apple IIGS pour implementer
 * une attente precise. GetTick() retourne le nombre de ticks ecoules
 * depuis le demarrage du systeme (60 Hz sur Apple IIGS).
 */
void delay(int seconds) {
    long startTick = GetTick();               // Temps de debut (en ticks)
    long ticksToWait = seconds * 60;          // Conversion: 60 ticks/seconde
    
    // Boucle d'attente active jusqu'a ecoulement du delai
    while (GetTick() - startTick < ticksToWait) {
        // Attente passive - le processeur reste disponible pour le systeme
    }
}

// ============================================================================
//                    FONCTIONS DE GESTION DU MODELE 3D
// ============================================================================

/**
 * CREATION D'UN NOUVEAU MODELE 3D
 * ===============================
 * 
 * Cette fonction alloue dynamiquement toutes les structures necessaires
 * pour un modele 3D. L'allocation dynamique est cruciale sur Apple IIGS
 * car la pile est limitee et ne peut pas contenir de gros tableaux.
 * 
 * STRATEGIE D'ALLOCATION:
 * 1. Allocation de la structure principale Model3D
 * 2. Allocation du tableau de vertices (MAX_VERTICES elements)
 * 3. Allocation du tableau de faces (MAX_FACES elements)
 * 4. En cas d'echec: liberation des allocations precedentes (cleanup)
 * 
 * GESTION D'ERREURS:
 * - Verification de chaque allocation
 * - Liberation automatique en cascade si echec partiel
 * - Retour NULL si impossible d'allouer
 */
Model3D* createModel3D(void) {
    // Etape 1: Allocation de la structure principale
    Model3D* model = (Model3D*)malloc(sizeof(Model3D));
    if (model == NULL) {
        return NULL;  // Echec allocation structure principale
    }
    
    // Etape 2: Allocation du tableau de vertices
    // Taille: MAX_VERTICES * sizeof(Vertex3D) octets
    model->vertices = (Vertex3D*)malloc(MAX_VERTICES * sizeof(Vertex3D));
    if (model->vertices == NULL) {
        free(model);  // Cleanup: liberer la structure principale
        return NULL;  // Echec allocation tableau vertices
    }
    
    // Etape 3: Allocation du tableau de faces
    // Taille: MAX_FACES * sizeof(Face3D) octets
    model->faces = (Face3D*)malloc(MAX_FACES * sizeof(Face3D));
    if (model->faces == NULL) {
        free(model->vertices);  // Cleanup: liberer tableau vertices
        free(model);            // Cleanup: liberer structure principale
        return NULL;            // Echec allocation tableau faces
    }
    
    // Etape 4: Initialisation des compteurs
    model->vertex_count = 0;    // Aucun vertex charge initialement
    model->face_count = 0;      // Aucune face chargee initialement
    
    return model;  // Succes: retourner le modele initialise
}

/**
 * DESTRUCTION D'UN MODELE 3D
 * ==========================
 * 
 * Cette fonction libere proprement toute la memoire allouee pour un
 * modele 3D. Elle suit le principe LIFO (Last In, First Out) pour
 * la liberation: derniers alloues = premiers liberes.
 * 
 * ORDRE DE LIBERATION:
 * 1. Tableau des faces (si alloue)
 * 2. Tableau des vertices (si alloue)  
 * 3. Structure principale
 * 
 * SECURITE:
 * - Verification NULL pour eviter les erreurs de segmentation
 * - Liberation selective selon les allocations reussies
 */
void destroyModel3D(Model3D* model) {
    // Verification pointeur principal
    if (model != NULL) {
        // Liberation tableau vertices (si alloue)
        if (model->vertices != NULL) {
            free(model->vertices);
        }
        
        // Liberation tableau faces (si alloue)
        if (model->faces != NULL) {
            free(model->faces);
        }
        
        // Liberation structure principale
        free(model);
    }
}

/**
 * CHARGEMENT COMPLET D'UN MODELE 3D
 * =================================
 * 
 * Cette fonction coordonne le chargement complet d'un fichier OBJ
 * en appellant successivement les fonctions de lecture des vertices
 * et des faces.
 * 
 * PIPELINE DE CHARGEMENT:
 * 1. Verification des parametres d'entree
 * 2. Lecture des vertices depuis le fichier
 * 3. Lecture des faces depuis le fichier  
 * 4. Mise a jour des compteurs dans la structure
 * 
 * GESTION D'ERREURS:
 * - Echec lecture vertices: arret immediat
 * - Echec lecture faces: avertissement mais continuation
 *   (modele vertices-seulement reste utilisable)
 */
int loadModel3D(Model3D* model, const char* filename) {
    // Verification des parametres d'entree
    if (model == NULL || filename == NULL) {
        return -1;  // Parametres invalides
    }
    
    // Etape 1: Lecture des vertices depuis le fichier OBJ
    model->vertex_count = readVertices(filename, model->vertices, MAX_VERTICES);
    if (model->vertex_count < 0) {
        return -1;  // Echec critique: impossible de lire les vertices
    }
    
    // Etape 2: Lecture des faces depuis le fichier OBJ
    model->face_count = readFaces(filename, model->faces, MAX_FACES);
    if (model->face_count < 0) {
        // Echec non-critique: modele vertices-seulement reste utilisable
        printf("\nAvertissement: Impossible de lire les faces\n");
        model->face_count = 0;  // Aucune face disponible
    }
    
    return 0;  // Succes: modele charge (avec ou sans faces)
}

// ============================================================================
//                    FONCTIONS D'INTERFACE UTILISATEUR
// ============================================================================

/**
 * SAISIE DES PARAMETRES DE L'OBSERVATEUR
 * ======================================
 * 
 * Cette fonction presente une interface textuelle pour permettre a 
 * l'utilisateur de specifier les parametres de visualisation 3D.
 * 
 * PARAMETRES DEMANDES:
 * - Angle horizontal: rotation autour de l'axe Y (vue gauche/droite)
 * - Angle vertical: rotation autour de l'axe X (vue haut/bas)
 * - Distance: eloignement de l'observateur (zoom)
 * - Angle rotation ecran: rotation finale dans le plan 2D
 * 
 * GESTION D'ERREURS:
 * - Valeurs par defaut si echec de saisie
 * - Conversion automatique string->float avec atof()
 */
void getObserverParams(ObserverParams* params) {
    char input[50];  // Buffer pour la saisie utilisateur
    
    // Affichage de l'en-tete de la section
    printf("\nParametres de l'observateur:\n");
    printf("============================\n");
    printf("(Appuyez ENTREE pour utiliser les valeurs par defaut)\n");
    printf("(Entrez 'debug' pour voir les valeurs utilisees)\n");
    
    // Saisie angle horizontal (rotation autour Y)
    printf("Angle horizontal (degres, defaut 30): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Enlever le saut de ligne
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_h = 30.0;     // Valeur par defaut si ENTREE
        } else {
            params->angle_h = atof(input);  // Conversion string->float
        }
    } else {
        params->angle_h = 30.0;         // Valeur par defaut si echec
    }
    
    // Saisie angle vertical (rotation autour X)  
    printf("Angle vertical (degres, defaut 15): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Enlever le saut de ligne
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_v = 15.0;     // Valeur par defaut si ENTREE
        } else {
            params->angle_v = atof(input);  // Conversion string->float
        }
    } else {
        params->angle_v = 15.0;         // Valeur par defaut si echec
    }
    
    // Saisie distance d'observation (zoom/perspective)
    printf("Distance (defaut 10): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Enlever le saut de ligne
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->distance = 10.0;    // Valeur par defaut si ENTREE
        } else {
            params->distance = atof(input); // Conversion string->float
        }
    } else {
        params->distance = 10.0;        // Distance par defaut: vue equilibree
    }
    
    // Saisie angle de rotation ecran (rotation finale 2D)
    printf("Angle de rotation ecran (degres, defaut 0): ");
    if (fgets(input, sizeof(input), stdin) != NULL) {
        // Enlever le saut de ligne
        input[strcspn(input, "\n")] = 0;
        if (strlen(input) == 0) {
            params->angle_w = 0.0;      // Valeur par defaut si ENTREE
        } else {
            params->angle_w = atof(input);  // Conversion string->float
        }
    } else {
        params->angle_w = 0.0;          // Pas de rotation par defaut
    }
    
//     // DEBUG: Afficher les parametres finalement utilisés
//     printf("\n=== PARAMETRES UTILISES ===\n");
//     printf("Angle horizontal: %.1f\n", params->angle_h);
//     printf("Angle vertical: %.1f\n", params->angle_v);
//     printf("Distance: %.1f\n", params->distance);
//     printf("Angle ecran: %.1f\n", params->angle_w);
//     printf("==========================\n");
//     keypress();
}

/**
 * AFFICHAGE DES INFORMATIONS DU MODELE
 * ====================================
 * 
 * Affiche un resume statistique du modele 3D charge:
 * nombre de vertices et nombre de faces.
 * 
 * Cette fonction sert de validation rapide que le chargement
 * s'est deroule correctement.
 */
void displayModelInfo(Model3D* model) {
    printf("\nResume de l'analyse:\n");
    printf("====================\n");
    printf("Nombre de vertices (points 3D) trouves: %d\n", model->vertex_count);
    printf("Nombre de faces trouvees: %d\n", model->face_count);
}

/**
 * AFFICHAGE DETAILLE DES RESULTATS
 * ================================
 * 
 * Cette fonction affiche toutes les coordonnees des vertices
 * a travers les differentes etapes du pipeline de rendu:
 * 1. Coordonnees originales (x, y, z)
 * 2. Coordonnees transformees (xo, yo, zo) 
 * 3. Coordonnees projetees 2D (x2d, y2d)
 * 
 * Egalement affiche la liste des faces avec leurs vertices.
 * Utile pour le debug et la verification des calculs.
 */
void displayResults(Model3D* model) {
    int i, j;
    
    // Affichage des vertices si presents
    if (model->vertex_count > 0) {
        printf("\nCoordonnees completes (Originales -> 3D -> 2D):\n");
        printf("-----------------------------------------------\n");
        
        // Parcours de tous les vertices
        for (i = 0; i < model->vertex_count; i++) {
            // Verification si le vertex est visible a l'ecran
            if (model->vertices[i].x2d >= 0 && model->vertices[i].y2d >= 0) {
                // Vertex visible: afficher toutes les coordonnees
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (%d,%d)\n", 
                       i + 1,  // Numerotation base 1 pour l'utilisateur
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,     // Originales
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo,  // Transformees
                       model->vertices[i].x2d, model->vertices[i].y2d);                      // Projetees 2D
            } else {
                // Vertex invisible (derriere observateur ou hors ecran)
                printf("  Vertex %3d: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) -> (invisible)\n", 
                       i + 1,  // Numerotation base 1 pour l'utilisateur
                       model->vertices[i].x, model->vertices[i].y, model->vertices[i].z,     // Originales
                       model->vertices[i].xo, model->vertices[i].yo, model->vertices[i].zo); // Transformees
            }
        }
    }
    
    // Affichage des faces si presentes
    if (model->face_count > 0) {
        printf("\nListe des faces:\n");
        printf("----------------\n");
        
        // Parcours de toutes les faces
        for (i = 0; i < model->face_count; i++) {
            printf("  Face %3d (%d vertices, z_max=%.2f): ", i + 1, model->faces[i].vertex_count, model->faces[i].z_max);
            
            // Affichage des indices des vertices de cette face
            for (j = 0; j < model->faces[i].vertex_count; j++) {
                printf("%d", model->faces[i].vertex_indices[j]); // Index base 1
                if (j < model->faces[i].vertex_count - 1) printf("-"); // Separateur
            }
            printf("\n");
            
            // Affichage detaille des coordonnees des vertices de cette face
            printf("       Coordonnees des vertices de cette face:\n");
            for (j = 0; j < model->faces[i].vertex_count; j++) {
                int vertex_idx = model->faces[i].vertex_indices[j] - 1; // Convertir base 1 vers base 0
                if (vertex_idx >= 0 && vertex_idx < model->vertex_count) {
                    printf("         Vertex %d: (%.2f,%.2f,%.2f) -> (%d,%d)\n",
                           model->faces[i].vertex_indices[j], // Index original base 1
                           model->vertices[vertex_idx].x, model->vertices[vertex_idx].y, model->vertices[vertex_idx].z,
                           model->vertices[vertex_idx].x2d, model->vertices[vertex_idx].y2d);
                } else {
                    printf("         Vertex %d: ERREUR - Index hors limites!\n", 
                           model->faces[i].vertex_indices[j]);
                }
            }
            printf("\n");
        }
        
        // Lancement du rendu graphique (si faces disponibles)
        drawPolygons(model->vertices, model->faces, model->face_count, model->vertex_count);
    }
}

/**
 * TRAITEMENT COMPLET D'UN MODELE 3D
 * =================================
 * 
 * Cette fonction execute le pipeline complet de transformation 3D:
 * 1. Transformation vers le systeme observateur
 * 2. Projection sur l'ecran 2D
 * 
 * Elle coordonne les deux etapes principales du rendu 3D en
 * appellant les fonctions specialisees dans l'ordre correct.
 */
void processModel(Model3D* model, ObserverParams* params) {
    // Etape 1: Transformation 3D vers systeme observateur
    // Application des rotations et translation selon les parametres
    transformToObserver(model->vertices, model->vertex_count, 
                       params->angle_h, params->angle_v, params->distance);
    
    // Etape 1.5: Calcul des profondeurs max pour chaque face (pour tri Z-buffer)
    calculateFaceDepths(model->vertices, model->faces, model->face_count);
    
    // Etape 1.6: Tri des faces par profondeur (algorithme du peintre)
    sortFacesByDepth(model->faces, model->face_count);
    // printf("Faces triees par profondeur (%d faces)\n", model->face_count);
    
    // Etape 2: Projection perspective sur ecran 2D
    // Conversion 3D -> 2D avec gestion de la perspective
    projectTo2D(model->vertices, model->vertex_count, params->angle_w);
}

// ============================================================================
//                    IMPLEMENTATIONS DES FONCTIONS DE BASE
// ============================================================================

/**
 * LECTURE DES VERTICES DEPUIS UN FICHIER OBJ
 * ==========================================
 * 
 * Cette fonction parse un fichier au format OBJ pour extraire les
 * vertices (points 3D). Elle recherche les lignes commencant par "v "
 * et extrait les coordonnees X, Y, Z.
 * 
 * FORMAT OBJ POUR VERTICES:
 *   v 1.234 5.678 9.012
 *   v -2.5 0.0 3.14159
 * 
 * ALGORITHME:
 * 1. Ouverture du fichier en mode lecture
 * 2. Lecture ligne par ligne avec fgets()
 * 3. Detection des lignes "v " avec verification caracteres
 * 4. Extraction coordonnees avec sscanf()
 * 5. Stockage dans tableau avec verification limites
 * 6. Affichage progressif pour feedback utilisateur
 * 
 * GESTION D'ERREURS:
 * - Verification ouverture fichier
 * - Protection contre debordement tableau
 * - Validation format coordonnees
 */
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
        // printf("%3d: %s", line_number, line);
        
        // Verifier si la ligne commence par "v " (vertex - format OBJ standard)
        if (line[0] == 'v' && line[1] == ' ') {
            if (vertex_count < max_vertices) {
                float x, y, z;
                // Extraire les coordonnees x, y, z
                if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
                    vertices[vertex_count].x = x;
                    vertices[vertex_count].y = y;
                    vertices[vertex_count].z = z;
                    vertex_count++;
                    // printf("     -> Vertex %d: (%.3f, %.3f, %.3f)\n", 
                    //        vertex_count, x, y, z);
                }
            } else {
                printf("     -> ATTENTION: Limite de vertices atteinte (%d)\n", max_vertices);
            }
        }
        
        line_number++;
    }
    
    // Fermer le fichier
    fclose(file);
    
    // printf("\n\nAnalyse terminee. %d lignes lues.\n", line_number - 1);
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
                // printf("%3d: %s", line_number, line);
                
                faces[face_count].vertex_count = 0;
                
                // Utiliser une approche plus robuste que strtok
                char *ptr = line + 2;  // Commencer apres "f "
                faces[face_count].vertex_count = 0;
                
                // Analyser caractere par caractere
                while (*ptr != '\0' && *ptr != '\n' && faces[face_count].vertex_count < MAX_FACE_VERTICES) {
                    // Ignorer les espaces et tabulations
                    while (*ptr == ' ' || *ptr == '\t') ptr++;
                    
                    if (*ptr == '\0' || *ptr == '\n') break;
                    
                    // Lire le nombre
                    int vertex_index = 0;
                    while (*ptr >= '0' && *ptr <= '9') {
                        vertex_index = vertex_index * 10 + (*ptr - '0');
                        ptr++;
                    }
                    
                    // Ignorer les donnees de texture/normale (apres /)
                    while (*ptr != '\0' && *ptr != ' ' && *ptr != '\t' && *ptr != '\n') {
                        ptr++;
                    }
                    
                    // Verifier que l'indice est valide (base 1, donc >= 1)
                    if (vertex_index >= 1) {
                        faces[face_count].vertex_indices[faces[face_count].vertex_count] = vertex_index;
                        faces[face_count].vertex_count++;
                        // printf("  -> Vertex index lu: %d\n", vertex_index);
                    } else if (vertex_index > 0) {  // vertex_index == 0 signifie qu'aucun nombre n'a ete lu
                        printf("     -> AVERTISSEMENT: Indice de vertex invalide %d ignore\n", vertex_index);
                    }
                }
                
                // printf("     -> Face %d: %d vertices (", face_count + 1, faces[face_count].vertex_count);
                // for (i = 0; i < faces[face_count].vertex_count; i++) {
                //     printf("%d", faces[face_count].vertex_indices[i]);
                //     if (i < faces[face_count].vertex_count - 1) printf(",");
                // }
                // printf(")\n");
                
                // Validation supplementaire: si vertex count = 0, ignorer cette face
                if (faces[face_count].vertex_count == 0) {
                    printf("     -> AVERTISSEMENT: Face sans vertices valides ignoree\n");
                } else {
                    face_count++;
                }
            } else {
                printf("     -> ATTENTION: Limite de faces atteinte (%d)\n", max_faces);
            }
        }
        
        line_number++;
    }
    
    // Fermer le fichier
    fclose(file);
    
    // printf("\n\nAnalyse des faces terminee. %d faces lues.\n", face_count);
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
        
        // printf("Vertex %3d: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)\n", 
        //        i + 1, x, y, z, vertices[i].xo, vertices[i].yo, vertices[i].zo);
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
            
            // printf("Vertex %3d: 3D(%.2f,%.2f,%.2f) -> 2D(%d,%d)\n", 
            //        i + 1, vertices[i].xo, vertices[i].yo, vertices[i].zo, 
            //        vertices[i].x2d, vertices[i].y2d);
        } else {
            // Point derriere l'observateur, pas de projection
            vertices[i].x2d = -1;
            vertices[i].y2d = -1;
            // printf("Vertex %3d: Derriere l'observateur (zo=%.2f)\n", 
            //        i + 1, vertices[i].zo);
        }
    }
}

/**
 * CALCUL DES PROFONDEURS MAXIMALES DES FACES
 * ==========================================
 * 
 * Cette fonction calcule pour chaque face la profondeur maximale (z_max)
 * de tous ses vertices dans le systeme de coordonnees de l'observateur.
 * Cette valeur est utilisee pour le tri des faces lors du rendu
 * (algorithme du peintre).
 * 
 * PARAMETRES:
 *   vertices   : Tableau des vertices avec coordonnees dans le systeme observateur
 *   faces      : Tableau des faces a traiter
 *   face_count : Nombre de faces
 * 
 * NOTES:
 *   - Doit etre appelee APRES transformToObserver()
 *   - Utilise les coordonnees zo (systeme observateur)
 *   - Plus la valeur z_max est grande, plus la face est eloignee
 */
void calculateFaceDepths(Vertex3D* vertices, Face3D* faces, int face_count) {
    int i, j;
    
    // Pour chaque face
    for (i = 0; i < face_count; i++) {
        float z_max = -9999.0;  // Initialiser avec une valeur tres petite
        
        // Parcourir tous les vertices de cette face
        for (j = 0; j < faces[i].vertex_count; j++) {
            int vertex_idx = faces[i].vertex_indices[j] - 1; // Conversion base 1 vers base 0
            
            // Verifier que l'indice est valide (utiliser un nombre raisonnable de vertices)
            if (vertex_idx >= 0) {
                // Comparer avec la coordonnee zo (profondeur dans systeme observateur)
                if (vertices[vertex_idx].zo > z_max) {
                    z_max = vertices[vertex_idx].zo;
                }
            }
        }
        
        // Stocker la profondeur maximale dans la face
        faces[i].z_max = z_max;
        
        // Debug optionnel
        // printf("Face %d: z_max = %.2f\n", i + 1, z_max);
    }
}

/**
 * TRI DES FACES PAR PROFONDEUR (VERSION OPTIMISEE)
 * ================================================
 * 
 * Cette fonction trie les faces par ordre decroissant de profondeur z_max
 * pour implementer l'algorithme du peintre. Les faces les plus eloignees
 * (z_max plus grand) sont placees en premier dans le tableau pour etre
 * dessinees en premier, et les plus proches en dernier.
 * 
 * PARAMETRES:
 *   faces      : Tableau des faces a trier
 *   face_count : Nombre de faces
 * 
 * ALGORITHMES:
 *   - Tri par insertion pour petites collections (< 10 faces) - O(n²) mais rapide
 *   - Tri rapide (quicksort) pour grandes collections - O(n log n) en moyenne
 *   - Detection des tableaux deja tries pour eviter le tri inutile - O(n)
 * 
 * OPTIMISATIONS:
 *   - Seuil adaptatif selon la taille
 *   - Verification pre-tri pour tableaux deja ordonnes
 *   - Pivotage median-de-trois pour quicksort stable
 * 
 * NOTES:
 *   - Doit etre appelee APRES calculateFaceDepths()
 *   - Ordre de tri: z_max decroissant (plus eloigne vers plus proche)
 */
void sortFacesByDepth(Face3D* faces, int face_count) {
    int i;
    
    // Cas trivial: 0 ou 1 face
    if (face_count <= 1) {
        return;
    }
    
    // Optimisation 1: Verifier si le tableau est deja trie (cas frequent en 3D)
    int already_sorted = 1;
    for (i = 0; i < face_count - 1; i++) {
        if (faces[i].z_max < faces[i + 1].z_max) {
            already_sorted = 0;
            break;
        }
    }
    if (already_sorted) {
        // printf("Tableau deja trie - optimisation activee\n");
        return;  // Deja trie, pas de travail a faire
    }
    
    // Optimisation 2: Choix algorithmique selon la taille
    if (face_count <= 10) {
        // Tri par insertion optimise pour petites collections
        printf("Tri insertion (petite collection: %d faces)\n", face_count);
        sortFacesByDepth_insertion(faces, face_count);
    } else {
        // Tri rapide pour grandes collections
        printf("Tri rapide (grande collection: %d faces)\n", face_count);
        sortFacesByDepth_quicksort(faces, 0, face_count - 1);
    }
    
    // Debug optionnel - afficher l'ordre de tri
    /*
    printf("Ordre de tri des faces (plus eloigne -> plus proche):\n");
    for (i = 0; i < face_count; i++) {
        printf("  Position %d: Face avec z_max=%.2f\n", i + 1, faces[i].z_max);
    }
    */
}

/**
 * TRI PAR INSERTION OPTIMISE (pour petites collections)
 * =====================================================
 */
void sortFacesByDepth_insertion(Face3D* faces, int face_count) {
    int i, j;
    Face3D temp_face;
    
    for (i = 1; i < face_count; i++) {
        // Optimisation: verifier si l'element est deja a sa place
        if (faces[i].z_max <= faces[i - 1].z_max) {
            continue;  // Deja bien place
        }
        
        // Sauvegarder la face courante
        temp_face = faces[i];
        
        // Decaler les faces avec z_max plus petit vers la droite
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
    float pivot = faces[low].z_max;
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

// Fonction pour dessiner les polygones avec QuickDraw
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
    // Dessiner chaque face
    for (i = 0; i < face_count; i++) {
        // Dessiner toutes les faces avec au moins 3 vertices
        if (faces[i].vertex_count >= 3) {
            
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
                    int vertex_idx = faces[i].vertex_indices[j] - 1; // Convertir base 1 vers base 0
                    
                    poly->polyPoints[j].h = mode / 320 * vertices[vertex_idx].x2d;
                    poly->polyPoints[j].v = vertices[vertex_idx].y2d;
                    
                    // Mettre a jour la bounding box
                    if (min_x == -1 || vertices[vertex_idx].x2d < min_x) min_x = vertices[vertex_idx].x2d;
                    if (max_x == -1 || vertices[vertex_idx].x2d > max_x) max_x = vertices[vertex_idx].x2d;
                    if (min_y == -1 || vertices[vertex_idx].y2d < min_y) min_y = vertices[vertex_idx].y2d;
                    if (max_y == -1 || vertices[vertex_idx].y2d > max_y) max_y = vertices[vertex_idx].y2d;
                }
                
                // Definir la bounding box
                poly->polyBBox.h1 = min_x;
                poly->polyBBox.v1 = min_y;
                poly->polyBBox.h2 = max_x;
                poly->polyBBox.v2 = max_y;
                
                // Definir la couleur (cyclique pour varier les couleurs)
                //SetSolidPenPat((i % 15) + 1);

                // Dessiner le polygone
                SetSolidPenPat(14);     // gris clair
                GetPenPat(pat);
                FillPoly(polyHandle,pat);
                SetSolidPenPat(7);      // rouge
                FramePoly(polyHandle);
                
                // Nettoyer
                HUnlock(polyHandle);
                DisposeHandle(polyHandle);
                
                valid_faces_drawn++;
            } else {
                // printf("Erreur: Impossible d'allouer la memoire pour la face %d\n", i + 1);
                invalid_faces_skipped++;
            }
        } else {
            // printf("Face %d ignoree (moins de 3 vertices: %d)\n", i + 1, faces[i].vertex_count);
            invalid_faces_skipped++;
        }
    }
    
    // printf("=== RESUME DU RENDU ===\n");
    // printf("Faces dessinees avec succes: %d\n", valid_faces_drawn);
    // printf("Faces ignorees/invalides: %d\n", invalid_faces_skipped);
    // printf("========================\n");
}

/**
 * SAUVEGARDE DES DONNEES DE DEBUG
 * ==============================
 * 
 * Cette fonction sauvegarde toutes les donnees du modele 3D dans un fichier
 * texte pour permettre l'analyse des problemes d'affichage.
 * 
 * CONTENU DU FICHIER DEBUG:
 * - Liste complete des vertices avec coordonnees 3D et 2D
 * - Liste complete des faces avec indices et coordonnees calculees
 * - Statistiques du modele
 */
void saveDebugData(Model3D* model, const char* debug_filename) {
    FILE *debug_file;
    int i, j;
    
    // Ouvrir le fichier de debug en ecriture
    debug_file = fopen(debug_filename, "w");
    if (debug_file == NULL) {
        printf("Erreur: Impossible de creer le fichier de debug '%s'\n", debug_filename);
        return;
    }
    
    // printf("\n=== SAUVEGARDE DEBUG ===\n");
    // printf("Ecriture dans: %s\n", debug_filename);
    
    // En-tete du fichier
    fprintf(debug_file, "=== DONNEES DE DEBUG DU MODELE 3D ===\n");
    fprintf(debug_file, "Date de generation: %s", __DATE__);
    fprintf(debug_file, "\n\n");
    
    // Statistiques generales
    fprintf(debug_file, "=== STATISTIQUES ===\n");
    fprintf(debug_file, "Vertices charges: %d\n", model->vertex_count);
    fprintf(debug_file, "Faces chargees: %d\n", model->face_count);
    fprintf(debug_file, "\n");
    
    // Analyse des faces par nombre de vertices
    int triangle_count = 0, quad_count = 0, other_count = 0;
    for (i = 0; i < model->face_count; i++) {
        if (model->faces[i].vertex_count == 3) triangle_count++;
        else if (model->faces[i].vertex_count == 4) quad_count++;
        else other_count++;
    }
    fprintf(debug_file, "Triangles detectes: %d\n", triangle_count);
    fprintf(debug_file, "Quadrilateres detectes: %d\n", quad_count);
    fprintf(debug_file, "Autres polygones: %d\n", other_count);
    fprintf(debug_file, "\n");
    
    // Liste complete des vertices
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
    
    // Liste complete des faces
    fprintf(debug_file, "=== FACES ===\n");
    for (i = 0; i < model->face_count; i++) {
        fprintf(debug_file, "Face F%03d (%d vertices):\n", i + 1, model->faces[i].vertex_count);
        fprintf(debug_file, "  Indices: ");
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            fprintf(debug_file, "V%d", model->faces[i].vertex_indices[j]);
            if (j < model->faces[i].vertex_count - 1) fprintf(debug_file, ", ");
        }
        fprintf(debug_file, "\n");
        
        // Coordonnees 3D et 2D de chaque vertex de la face
        fprintf(debug_file, "  Coordonnees:\n");
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            int vertex_idx = model->faces[i].vertex_indices[j] - 1; // Conversion base-1 vers base-0
            if (vertex_idx >= 0 && vertex_idx < model->vertex_count) {
                fprintf(debug_file, "    V%d: 3D(%.3f, %.3f, %.3f) -> 2D(%d, %d)\n",
                        model->faces[i].vertex_indices[j],
                        model->vertices[vertex_idx].x, model->vertices[vertex_idx].y, model->vertices[vertex_idx].z,
                        model->vertices[vertex_idx].x2d, model->vertices[vertex_idx].y2d);
            } else {
                fprintf(debug_file, "    V%d: ERREUR - Index hors limites!\n", model->faces[i].vertex_indices[j]);
            }
        }
        fprintf(debug_file, "\n");
    }
    
    // Verification d'integrite
    fprintf(debug_file, "=== VERIFICATION D'INTEGRITE ===\n");
    int errors = 0;
    for (i = 0; i < model->face_count; i++) {
        for (j = 0; j < model->faces[i].vertex_count; j++) {
            int vertex_idx = model->faces[i].vertex_indices[j] - 1;
            if (vertex_idx < 0 || vertex_idx >= model->vertex_count) {
                fprintf(debug_file, "ERREUR: Face F%d reference vertex V%d inexistant (index %d hors limites [1-%d])\n",
                        i + 1, model->faces[i].vertex_indices[j], vertex_idx + 1, model->vertex_count);
                errors++;
            }
        }
    }
    if (errors == 0) {
        fprintf(debug_file, "Aucune erreur detectee - Tous les indices sont valides.\n");
    } else {
        fprintf(debug_file, "TOTAL: %d erreurs detectees!\n", errors);
    }
    
    // Fermer le fichier
    fclose(debug_file);
    // printf("Debug sauvegarde avec succes!\n");
    // printf("========================\n");
}


void DoColor() {
        Rect r;
        unsigned char pstr[4];  // Chaîne Pascal: [longueur][caractères...]

        SetRect (&r, 0, 10, mode / 320 *10, 20);
        for (int i = 0; i < 16; i++) {
            SetSolidPenPat(i);
            PaintRect(&r);

            if (i == 0) {
                SetSolidPenPat(15); // Blanc pour le fond noir
                FrameRect(&r);
            }

            MoveTo(r.h1, r.v2+10);
            // Créer une chaîne Pascal pour afficher le numéro
            if (i < 10) {
                pstr[0] = 1;           // Longueur: 1 caractère
                pstr[1] = '0' + i;     // Le chiffre 0-9
            } else {
                pstr[0] = 2;           // Longueur: 2 caractères
                pstr[1] = '0' + (i / 10);      // Dizaine (1 pour 10-15)
                pstr[2] = '0' + (i % 10);      // Unité (0-5 pour 10-15)
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
    
    bigloop:
    // Traiter le modele avec les parametres
    processModel(model, &params);
    
    // Sauvegarder les donnees de debug pour analyse
    saveDebugData(model, "debug.txt");
    
    // Afficher les informations et resultats
    // displayModelInfo(model);
    // displayResults(model);

    loopReDraw:
    if (model->face_count > 0) {
    // Initialiser QuickDraw
    startgraph(mode);
    // Dessiner l'objet 3D
    drawPolygons(model->vertices, model->faces, model->face_count, model->vertex_count);
    // affiche les couleurs disponibles
    DoColor(); 
    keypress();
    }

    int key = 0;
    endgraph();
    DoText();

asm 
        {
loop:
        sep #0x20
        lda >0xC000     // Read the keyboard status from memory address 0xC000
        beq loop        // If not pressed, loop until a key is pressed
        sta >0xC010     // Clear the keypress by writing back to 0xC010
        sta key         // Store the key code in variable 'key'
        rep #0x30
        }
    sprintf(input, "You pressed key code: %d\n", key);
    printf("%s", input);
    if (key == 32) // space bar to redraw
        goto loopReDraw;
    else if (key == 65 || key == 97) // 'A pour diminuer la distance
    {
        params.distance = params.distance - (params.distance / 10); // just an example modification
        goto bigloop;
    }
    else if (key == 90 || key == 122) // 'Z' pour augmenter la distance
    {
        params.distance = params.distance + (params.distance / 10); 
        goto bigloop;
    }
    else if (key == 21 ) // fleche droite pour augmenter l'angle horizontal
    {
        params.angle_h = params.angle_h + 10; 
        goto bigloop;
    }
    else if (key == 8) // fleche gauche pour diminuer l'angle horizontal
    {
        params.angle_h = params.angle_h - 10; 
        goto bigloop;
    }
    else if (key == 10 ) // 'W' pour diminuer l'angle vertical
    {
        params.angle_v = params.angle_v - 10; 
        goto bigloop;
    }
    else if (key == 11 ) // 'S' pour augmenter l'angle vertical
    {
        params.angle_v = params.angle_v + 10; 
        goto bigloop;
    }
    else if (key == 8 ) // 
    {
        params.angle_v = params.angle_v - 10; 
        goto bigloop;
    }
    else if (key == 87 || key == 119)  // 'w' pour diminuer l'angle de rotation de l'ecran
    {
        params.angle_w = params.angle_w - 10; 
        goto bigloop;
    }
    else if (key == 88 || key == 120)  // 'x' pour augmenter l'angle de rotation de l'ecran
    {
        params.angle_w = params.angle_w + 10; 
        goto bigloop;
    }

    else if (key == 27) // 'ESC' pour quitter
    {
        goto end;
  }

    else goto loopReDraw; // toutes les autres touches pour redessiner

    end:
    // Nettoyage et fin
    destroyModel3D(model);
    return 0;
}

