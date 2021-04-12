#ifndef LINE_BASED_RECONS_REFACTO_LINESET_H
#define LINE_BASED_RECONS_REFACTO_LINESET_H

#include <limits>
#include "jsonTools.h"
#include "ObjectSet.h"
#include "Line.h"

class LineSet : public ObjectSet<Line>
{
public:
    LineSet();

    /* Insert a line from a json object */
    void insert(const nlohmann::json &lineJson);
    
    /* Getter */
    CGAL::Bbox_3 getBbox(double factor=0.) const;

    /* Setter */
    void setMaxNbPlanes(int _maxNbPlanes);

private:
    CGAL::Bbox_3 bbox;
    int maxNbPlanes;
};


#endif //LINE_BASED_RECONS_REFACTO_LINESET_H
