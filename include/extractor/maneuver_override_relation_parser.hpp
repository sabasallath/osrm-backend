#ifndef MANEUVER_OVERRIDE_RELATION_PARSER_HPP
#define MANEUVER_OVERRIDE_RELATION_PARSER_HPP

#include "maneuver_override.hpp"

#include <boost/optional.hpp>
#include <string>
#include <vector>

namespace osmium
{
class Relation;
}

namespace osrm
{
namespace extractor
{

class ScriptingEnvironment;

/**
 * Parses the relations that represents turn restrictions.
 *
 * Currently only restrictions where the via objects is a node are supported.
 *  from   via   to
 * ------->(x)-------->
 *
 * While this class does not directly invoke any lua code _per relation_ it does
 * load configuration values from the profile, that are saved in variables.
 * Namely ```use_turn_restrictions``` and ```get_restrictions```.
 *
 * The restriction is represented by the osm id of the from way, the osm id of the
 * to way and the osm id of the via node. This representation must be post-processed
 * in the extractor to work with the edge-based data-model of OSRM:
 * Since the from and to way share the via-node a turn will have the following form:
 * ...----(a)-----(via)------(b)----...
 * So it can be represented by the tripe (a, via, b).
 */
class ManeuverOverrideRelationParser
{
  public:
    ManeuverOverrideRelationParser();
    boost::optional<InputManeuverOverride> TryParse(const osmium::Relation &relation) const;
};
}
}

#endif /* RESTRICTION_PARSER_HPP */
