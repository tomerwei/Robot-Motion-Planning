#ifndef KD_TREE_H
#define KD_TREE_H

#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_d.h>
#include <CGAL/K_neighbor_search.h>
#include <CGAL/Cartesian_d.h>

#define MAKE_POINTS_UNIQUE 0 

template <typename Kernel, typename Point>
class Compare_distance_to_point {
private:
  Point q;

public:
  Compare_distance_to_point(const Point& q_) : q(q_) {}

  bool operator()(const Point& p1, const Point& p2) const 
  {
    return ((CGAL::compare_distance_to_point(q, p1, p2) == CGAL::SMALLER ) ?
            true : false);
  }
}; //Compare_distance_to_point

template <typename _Kernel, typename _Point, typename _Traits>
class Kd_tree_
{
protected:
  typedef _Kernel                                   Kernel;
  typedef _Point                                    Point;
  typedef _Traits                                   Tree_traits;

private:
  typedef CGAL::K_neighbor_search<Tree_traits>      Neighbor_search;
  typedef typename Neighbor_search::Tree            Cgal_kd_tree;

private:
  std::vector<Point>          un_inserted_points;
  std::vector<Cgal_kd_tree*>  trees;
  unsigned int max_num_for_linear_search;
public:
  //constructor
  Kd_tree_(unsigned int max_num_for_linear_search_ = 500) 
    : max_num_for_linear_search(max_num_for_linear_search_)
  {}

  void insert(const Point& p)
  {
#if MAKE_POINTS_UNIQUE
    if (nearest_neighbor(p) == p) return;
#endif
    un_inserted_points.push_back(p);
    if (un_inserted_points.size() > max_num_for_linear_search) rearrange();
    return;
  }

  template <typename InputIterator>
  void insert(const InputIterator& begin, const InputIterator& end)
  {
#if MAKE_POINTS_UNIQUE
    std::vector<Point> pts;
    for (InputIterator iter = begin; iter != end; ++iter)
      if (nearest_neighbor(*iter) != *iter)
        pts.push_back(*iter);
    un_inserted_points.insert(un_inserted_points.end(), pts.begin(), pts.end());
#else
    un_inserted_points.insert(un_inserted_points.end(), begin, end);
#endif
    if (un_inserted_points.size() > max_num_for_linear_search)
      rearrange();
    return;
  }

  Point nearest_neighbor(const Point& q) 
  {
    if (un_inserted_points.empty() && trees.empty()) return Point();
    Point nn;
    unsigned int trees_indx;
    if (un_inserted_points.empty() == false) {
      nn = nearest_neighbor(un_inserted_points, q);
      trees_indx = 0;
    }
    else {
      nn = nearest_neighbor(trees[0], q);
      trees_indx = 1;
    }

    typename Kernel::FT sq_dist = CGAL::squared_distance(nn, q);
    for (unsigned int i(trees_indx); i < trees.size(); ++i) {
      Point tmp_nn = nearest_neighbor(trees[i], q);
      typename Kernel::FT tmp_sq_dist = CGAL::squared_distance(tmp_nn, q);
      if (tmp_sq_dist < sq_dist) {
        sq_dist = tmp_sq_dist;
        nn = tmp_nn;
      }
    }
    return nn;
  }
  template <typename OutputIterator>
  void k_nearest_neighbors(        const Point& q, int k, 
                                   OutputIterator oi)
  {
    if (un_inserted_points.empty() && trees.empty()) return;

    std::vector<Point> nn;
          
    if (un_inserted_points.empty() == false)
      k_nearest_neighbors(un_inserted_points, q, k, std::back_inserter(nn));

    for (unsigned int i(0); i < trees.size(); ++i) {
      std::vector<Point> tmp_k_nn;
      k_nearest_neighbors(trees[0], q, k, std::back_inserter(tmp_k_nn));
                  
      //merge tmp_k_nn and nn
      k_merge(nn, tmp_k_nn, q, k);
    }

    BOOST_FOREACH(Point p, nn) *oi ++ = p;
    return ;
  }

public:
  template <typename OutputIterator>
  void points(OutputIterator& oi)
  {
    BOOST_FOREACH(Point p, un_inserted_points)
      *oi++ = p;
    BOOST_FOREACH(Cgal_kd_tree* tree, trees) {
      typename Cgal_kd_tree::iterator iter;
      for (iter = tree->begin(); iter!= tree->end(); ++iter)
        *oi++ = *iter;
    }
    return;
  }
private:
  void rearrange()
  {
    CGAL_precondition(un_inserted_points.size() > max_num_for_linear_search);
    if (trees.empty())
    {
      //collect points 
      Cgal_kd_tree* cgal_kd_tree_ptr= new Cgal_kd_tree();
      cgal_kd_tree_ptr->insert(un_inserted_points.begin(),
                               un_inserted_points.end());
      un_inserted_points.clear();
      trees.push_back(cgal_kd_tree_ptr);
      return;
    }

    unsigned int num_of_trees = trees.size();
    unsigned int min_tree_index_to_merge = trees.size() - 1;
    unsigned int num_of_elems =
      un_inserted_points.size() + trees[min_tree_index_to_merge]->size();
    while (min_tree_index_to_merge > 0) {
      if (trees[min_tree_index_to_merge-1]->size() <= num_of_elems) {
        num_of_elems += trees[min_tree_index_to_merge-1]->size();
        min_tree_index_to_merge--;
      }
      else
        break;
    }
    Cgal_kd_tree* cgal_kd_tree_ptr = trees[min_tree_index_to_merge];
    cgal_kd_tree_ptr->insert(un_inserted_points.begin(),
                             un_inserted_points.end());
    un_inserted_points.clear();
    for (unsigned int i(0); i < num_of_trees - min_tree_index_to_merge -1; ++i)
    {
      Cgal_kd_tree* cgal_kd_tree_ptr_back = trees.back();
      cgal_kd_tree_ptr->insert(cgal_kd_tree_ptr_back->begin(),
                               cgal_kd_tree_ptr_back->end());
      trees.pop_back();
      delete cgal_kd_tree_ptr_back;
    }
    return;
  }

  Point nearest_neighbor(const std::vector<Point>& points_vec, const Point& q)
  {
    typename Kernel::FT sq_d = CGAL::squared_distance(points_vec.front(), q);
    Point nn = points_vec.front();
    
    for (unsigned int i(1); i < points_vec.size(); ++i) {
      typename Kernel::FT tmp_sq_d = CGAL::squared_distance(q, points_vec[i]);
      if (CGAL::compare(tmp_sq_d, sq_d) == CGAL::SMALLER) {
        sq_d = tmp_sq_d;
        nn = points_vec[i];
      }
    }
    return nn;
  }

  Point nearest_neighbor(Cgal_kd_tree* cgal_kd_tree_ptr, const Point& q) 
  {
    Neighbor_search search(*cgal_kd_tree_ptr, q);
    return search.begin()->first;
  }

private:
  template <typename OutputIterator>
  void k_nearest_neighbors(std::vector<Point>& points_vec, const Point& q,
                           unsigned int k, OutputIterator oi)
  {
    Compare_distance_to_point<Kernel, Point> compare_distance_to_point(q);
    std::sort(points_vec.begin(), points_vec.end(), compare_distance_to_point);

    unsigned int k_ = std::min(k, points_vec.size());
    for (unsigned int i(0); i <k_; ++i)
      *oi ++ = points_vec[i];
                  
    return;
  }

  template <typename OutputIterator>
  void k_nearest_neighbors(Cgal_kd_tree* cgal_kd_tree_ptr, const Point& q,
                           unsigned int k, OutputIterator oi)
  {
    Neighbor_search search(*cgal_kd_tree_ptr, q, k);
    typename Neighbor_search::iterator iter;
    for (iter = search.begin(); iter != search.end(); ++iter)
      *oi++ = iter->first;
    return;
  }

  void k_merge(std::vector<Point>& original_points_vec, 
               std::vector<Point>& additional_points_vec, 
               const Point& q, unsigned int k)
  {
    std::vector<Point> merged(original_points_vec.size() +
                              additional_points_vec.size());
    Compare_distance_to_point<Kernel, Point> compare_distance_to_point(q);
    std::merge(original_points_vec.begin(), original_points_vec.end(),
               additional_points_vec.begin(), additional_points_vec.end(),
               merged.begin(), compare_distance_to_point);
          
    original_points_vec.clear();
    unsigned int k_ = std::min(k, merged.size());
    for (unsigned int i(0); i <k_; ++i)
      original_points_vec.push_back(merged[i]);

    return;          
  }
};  //Kd_tree_

template <typename K>
class Kd_tree_2 : public Kd_tree_<K, 
                                  typename K::Point_2, 
                                  typename CGAL::Search_traits_2<K> >
{
private:
  typedef Kd_tree_<K, 
                   typename K::Point_2, 
                   typename CGAL::Search_traits_2<K> >  Base;

public:
  Kd_tree_2(unsigned int max_num_for_linear_search_ = 500) :
    Base (max_num_for_linear_search_)
  {}
}; //Kd_tree_2

template <typename Kernel>
class Kd_tree_3 : public Kd_tree_<Kernel, 
                                  typename Kernel::Point_3, 
                                  typename CGAL::Search_traits_3<Kernel> >
{
private:
  typedef Kd_tree_<Kernel, 
                   typename Kernel::Point_3, 
                   typename CGAL::Search_traits_3<Kernel> >  Base;

public:
  Kd_tree_3(unsigned int max_num_for_linear_search_ = 8095) :
    Base (max_num_for_linear_search_)
  {}
}; //Kd_tree_3

#endif //KD_TREE_H
