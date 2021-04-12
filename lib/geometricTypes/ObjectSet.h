#ifndef LINE_BASED_RECONS_REFACTO_OBJECTSET_H
#define LINE_BASED_RECONS_REFACTO_OBJECTSET_H

#include <json/json.hpp>

/* Template pure virtual class for input objects storage such as Lines, Points
 * and Primitives. Ensures creation and destruction of these objects internally
 * and prevents unwanted copies of these objects.
 *
 * In order to implement an Object set, inherit from this class, with the type of
 * object as a template parameter, ensure that the child constructor calls the
 * ObjectSet constructor and implement the insert method.*/
template <typename T>
class ObjectSet
{
public:
    ObjectSet();
    virtual ~ObjectSet();

    /* Insert a line from a json object */
    virtual void insert(const nlohmann::json &objectJson) = 0;

    /* Get an object */
    const T& operator[](std::size_t idx) const;

    /* Get the set size */
    const std::size_t size() const;

    /* Tests if the set is empty */
    const bool empty() const;

protected:
    std::vector<T *> objects;

};

template <typename T>
ObjectSet<T>::ObjectSet() : objects(std::vector<T *>(0))
{

}

template <typename T>
ObjectSet<T>::~ObjectSet()
{
    for(auto objectIt : objects)
        delete objectIt;
}

template <typename T>
inline const T& ObjectSet<T>::operator[](std::size_t idx) const
{
    assert(idx >= 0);
    assert(idx < objects.size());
    return *objects[idx];
}

template <typename T>
inline const std::size_t ObjectSet<T>::size() const
{
    return objects.size();
}

template <typename T>
const bool ObjectSet<T>::empty() const
{
    return objects.empty();
}


#endif //LINE_BASED_RECONS_REFACTO_OBJECTSET_H
