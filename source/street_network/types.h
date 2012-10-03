#pragma once

#include "first_letter/first_letter.h"
#include "proximity_list/proximity_list.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <map>

namespace nt = navitia::type;

namespace navitia { namespace georef {

/** Propriétés Nœud (intersection entre deux routes) */
struct Vertex {
    type::GeographicalCoord coord;

    template<class Archive> void serialize(Archive & ar, const unsigned int) {
        ar & coord;
    }
};

/** Propriétés des arcs */

struct Edge {
    nt::idx_t way_idx; //< indexe vers le nom de rue
    float length; //< longeur en mètres de l'arc
    bool cyclable; //< est-ce que le segment est accessible à vélo ?

    template<class Archive> void serialize(Archive & ar, const unsigned int) {
        ar & way_idx & length & cyclable;// & start_number & end_number;
    }

    Edge() : way_idx(0), length(0), cyclable(false){}//, start_number(-1), end_number(-1){}
};

// Plein de typedefs pour nous simpfilier un peu la vie

/** Définit le type de graph que l'on va utiliser
  *
  * Les arcs sortants et la liste des nœuds sont représentés par des vecteurs
  * les arcs sont orientés
  * les propriétés des nœuds et arcs sont les classes définies précédemment
  */
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Graph;

/// Représentation d'un nœud dans le graphe
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;

/// Représentation d'un arc dans le graphe
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

/// Type Itérateur sur les nœuds du graphe
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;

/// Type itérateur sur les arcs du graphe
typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

// le numéro de la maison : il représente un point dans la rue, voie
struct HouseNumber{
    nt::GeographicalCoord coord;
    int number;
    HouseNumber(): number(-1){}

    bool operator<(const HouseNumber & other) const{
        return this->number < other.number;
    }

    template<class Archive> void serialize(Archive & ar, const unsigned int) {
        ar & coord & number;
    }
};

/** Nommage d'une voie (anciennement "adresse"). Typiquement le nom de rue **/
struct Way :public nt::Nameable{
    nt::idx_t idx;
    std::string way_type;
    std::string city;
    nt::idx_t city_idx;
    std::vector< HouseNumber > house_number_left;
    std::vector< HouseNumber > house_number_right;
    std::vector< std::pair<vertex_t, vertex_t> > edges;

    void sort_house_number();
    nt::GeographicalCoord get_geographicalCoord_by_number(int);
    nt::GeographicalCoord nearest_geographical_coord(int);    
    template<class Archive> void serialize(Archive & ar, const unsigned int) {
      ar & idx & name & comment & way_type & city & city_idx & house_number_left & house_number_right & edges;
    }
};


/** Un bout d'itinéraire : un nom de voie et une liste de segments */
struct PathItem{
    nt::idx_t way_idx; //< Voie sur laquel porte le bout du trajet
    float length; //< Longueur du trajet effectué sur cette voie
    std::vector<edge_t> segments; //< Segments traversés
    PathItem() : length(0){}
};

/** Itinéraire complet */
struct Path {
    float length; //< Longueur totale du parcours
    std::vector<PathItem> path_items; //< Liste des voies parcourues
    std::vector<nt::GeographicalCoord> coordinates; //< Coordonnées du parcours
};

/** Structure contenant tout ce qu'il faut savoir sur le référentiel de voirie */
struct GeoRef {

    /// Liste des voiries
    std::vector<Way> ways;

    /// Indexe sur les noms de voirie
    firstletter::FirstLetter<unsigned int> fl;

    /// Indexe
    proximitylist::ProximityList<vertex_t> pl;

    /// Graphe pour effectuer le calcul d'itinéraire
    Graph graph;

    template<class Archive> void save(Archive & ar, const unsigned int) const {
        ar & ways & graph & fl & pl;
    }

    template<class Archive> void load(Archive & ar, const unsigned int) {
        // La désérialisation d'une boost adjacency list ne vide pas le graphe
        // On avait donc une fuite de mémoire
        graph.clear();
        ar & ways & graph & fl & pl;
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()


    /** Construit l'indexe spatial */
    void build_proximity_list();

    /** Calcule le meilleur itinéraire entre deux listes de nœuds
     *
     * Le paramètre zeros indique la distances (en mètres) de chaque nœud de départ. Il faut qu'il y ait autant d'éléments que dans starts
     * Si la taille ne correspond pas, on considère une distance de 0
     */
    Path compute(std::vector<vertex_t> starts, std::vector<vertex_t> destinations, std::vector<double> start_zeros = std::vector<double>(), std::vector<double> dest_zeros = std::vector<double>()) const;

    /// Calcule le meilleur itinéraire entre deux coordonnées
    Path compute(const type::GeographicalCoord & start_coord, const type::GeographicalCoord & dest_coord) const;

    /** Retourne l'arc (segment) le plus proche
      *
      * Pour le trouver, on cherche le nœud le plus proche, puis pour chaque arc adjacent, on garde le plus proche
      * Ce n'est donc pas optimal, mais pour améliorer ça, il faudrait indexer des segments, ou ratisser plus large
     */

    edge_t nearest_edge(const type::GeographicalCoord &coordinates) const;
    edge_t nearest_edge(const type::GeographicalCoord &coordinates, const proximitylist::ProximityList<vertex_t> &prox) const;

    /** Initialise les structures nécessaires à dijkstra
     *
     * Attention !!! Modifie distances et predecessors
     **/
    void init(std::vector<float> & distances, std::vector<vertex_t> & predecessors) const;

    /** Lance un calcul de dijkstra sans initaliser de structure de données
     *
     * Attention !!! Modifie distances et predecessors
     **/
    template<class Visitor>
    void dijkstra(vertex_t start, std::vector<float> & distances, std::vector<vertex_t> & predecessors, Visitor visitor) const{
        boost::two_bit_color_map<> color(boost::num_vertices(this->graph));
        boost::dijkstra_shortest_paths_no_init(this->graph, start, &predecessors[0], &distances[0],

                                               boost::get(&Edge::length, this->graph), // weigth map
                                               boost::identity_property_map(),
                                               std::less<float>(), boost::closed_plus<float>(),
                                               0,
                                               visitor,
                                               color
                                               );

    }

    /// Reconstruit un itinéraire à partir de la destination et la liste des prédécesseurs
    Path build_path(vertex_t best_destination, std::vector<vertex_t> preds) const;
};


/** Lorsqu'on a une coordonnée, il faut l'accrocher au filaire. Cette structure contient l'accroche
  *
  * Elle consiste en deux nœuds possibles (les extrémités du segment où a été projeté la coordonnée)
  * La coordonnée projetée
  * Les distances entre le projeté les les nœuds
  */
struct ProjectionData {
    /// deux nœuds possibles (les extrémités du segment où a été projeté la coordonnée)
    vertex_t source;
    vertex_t target;

    /// Segment sur lequel a été projeté la coordonnée
    edge_t edge;

    /// La coordonnée projetée sur le segment
    type::GeographicalCoord projected;

    /// Distance entre le projeté et les nœuds du segment
    double source_distance;
    double target_distance;

    ProjectionData() : source_distance(-1), target_distance(-1){}
    /// Initialise la structure à partir d'une coordonnée et d'un graphe sur lequel on projette    
    ProjectionData(const type::GeographicalCoord & coord, const GeoRef &sn, const proximitylist::ProximityList<vertex_t> &prox);
};

/** Permet de construire un graphe de manière simple

    C'est essentiellement destiné aux tests unitaires
  */
struct GraphBuilder{
    /// Graphe que l'on veut construire
    //StreetNetwork & street_network;
    GeoRef & geo_ref;

    /// Associe une chaine de caractères à un nœud
    std::map<std::string, vertex_t> vertex_map;

    /// Le constructeur : on précise sur quel graphe on va construire
    //GraphBuilder(StreetNetwork & street_network) : street_network(street_network){}
    GraphBuilder(GeoRef & geo_ref) : geo_ref(geo_ref){}

    /// Ajoute un nœud, s'il existe déjà, les informations sont mises à jour
    GraphBuilder & add_vertex(std::string node_name, float x, float y);

    /// Ajoute un arc. Si un nœud n'existe pas, il est créé automatiquement
    /// Si la longueur n'est pas précisée, il s'agit de la longueur à vol d'oiseau
    GraphBuilder & add_edge(std::string source_name, std::string target_name, float length = -1, bool bidirectionnal = false);

    /// Surchage de la création de nœud pour plus de confort
    GraphBuilder & operator()(std::string node_name, float x, float y){ return add_vertex(node_name, x, y);}

    /// Surchage de la création de nœud pour plus de confort
    GraphBuilder & operator()(std::string node_name, navitia::type::GeographicalCoord geo){ return add_vertex(node_name, geo.x, geo.y);}

    /// Surchage de la création d'arc pour plus de confort
    GraphBuilder & operator()(std::string source_name, std::string target_name, float length = -1, bool bidirectionnal = false){
        return add_edge(source_name, target_name, length, bidirectionnal);
    }

    /// Retourne le nœud demandé, jette une exception si on ne trouve pas
    vertex_t get(const std::string & node_name);

    /// Retourne l'arc demandé, jette une exception si on ne trouve pas
    edge_t get(const std::string & source_name, const std::string & target_name);
};

/** Projette un point sur un segment

   Retourne les coordonnées projetées et la distance au segment
   Si le point projeté tombe en dehors du segment, alors ça tombe sur le nœud le plus proche
   http://paulbourke.net/geometry/pointline/
   */
std::pair<type::GeographicalCoord, float> project(type::GeographicalCoord point, type::GeographicalCoord segment_start, type::GeographicalCoord segment_end);

/** Structures avec toutes les données écriture pour streetnetwork */
struct StreetNetworkWorker {
public:
    //StreetNetworkWorker(const StreetNetwork & street_network);
    StreetNetworkWorker(const GeoRef & geo_ref);

    /** On définit les coordonnées de départ, un proximitylist et un rayon
     *
     * Retourne tous les idx atteignables dans ce rayon, ainsi que la distance en suivant le filaire de voirie
     **/
    std::vector< std::pair<type::idx_t, double> > find_nearest(const type::GeographicalCoord & start_coord, const proximitylist::ProximityList<type::idx_t> & pl, double radius, bool use_second=false);

    /// Reconstruit l'itinéraire piéton à partir de l'idx
    Path get_path(type::idx_t idx, bool use_second = false);

private:
    //const StreetNetwork & street_network;
    const GeoRef & geo_ref;

    std::vector< std::pair<type::idx_t, double> > find_nearest(const ProjectionData & start,
                                                               double radius,
                                                               const std::vector< std::pair<type::idx_t, type::GeographicalCoord> > & elements,
                                                               std::vector<float> & dist,
                                                               std::vector<vertex_t> & preds,
                                                               std::map<type::idx_t, ProjectionData> & idx_proj);

    // Les données sont doublées pour garder les données au départ et à l'arrivée
    /// Tableau des distances utilisé par Dijkstra
    std::vector<float> distances;
    std::vector<float> distances2;

    /// Tableau des prédécesseurs utilisé par Dijkstra
    std::vector<vertex_t> predecessors;
    std::vector<vertex_t> predecessors2;

    /// Associe chaque idx_t aux données de projection sur le filaire associées
    std::map<type::idx_t, ProjectionData> idx_projection;
    std::map<type::idx_t, ProjectionData> idx_projection2;
};

}} //namespace navitia::streetnetwork
