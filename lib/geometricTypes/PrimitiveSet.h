#ifndef LINE_BASED_RECONS_REFACTO_PRIMITIVESET_H
#define LINE_BASED_RECONS_REFACTO_PRIMITIVESET_H

#include "jsonTools.h"
#include "LineSet.h"
#include "Primitive.h"

class PrimitiveSet : public ObjectSet<Primitive>
{
public:
    PrimitiveSet();

    /* Insert a primitive from a json object */
    void insert(const nlohmann::json &primitiveJson);

    int orientatePrimitives(const LineSet& lineSet);

private:
};


#endif //LINE_BASED_RECONS_REFACTO_PRIMITIVESET_H
