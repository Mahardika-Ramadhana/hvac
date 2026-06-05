#ifndef ROBOT_ESR_V2_BALANCE_MARKERS_HPP
#define ROBOT_ESR_V2_BALANCE_MARKERS_HPP

#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/SceneMarkers>
#include <cnoid/SceneBody>
#include <cnoid/Link>
#include <cnoid/SceneGraph>

namespace cnoid { class BodyItem; }

struct BalanceMetrics {
    cnoid::Vector3 com  = cnoid::Vector3::Zero();
    cnoid::Vector3 cog  = cnoid::Vector3::Zero();
    cnoid::Vector3 cop  = cnoid::Vector3::Zero();
    cnoid::Vector3 comXY = cnoid::Vector3::Zero();
    double totalContactForce = 0.0;
    int    numContacts = 0;
    bool copValid = false;
};

class BalanceMarkers
{
public:
    void setup(cnoid::BodyItem* bodyItem);
    void clear();
    BalanceMetrics update(cnoid::Body* body);

private:
    cnoid::BodyItem* bodyItem_ = nullptr;
    cnoid::SgGroupPtr root_;
    cnoid::SphereMarkerPtr comMarker_;
    cnoid::SphereMarkerPtr cogMarker_;
    cnoid::SphereMarkerPtr copMarker_;
    cnoid::SphereMarkerPtr comProjMarker_;
    bool attached_ = false;

    static cnoid::Vector3 computeCoP(cnoid::Body* body, bool& valid,
                                     double& totalForce, int& numContacts);
};

#endif
