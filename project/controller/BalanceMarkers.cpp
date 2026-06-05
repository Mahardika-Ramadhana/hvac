#include "BalanceMarkers.hpp"

#include <cnoid/BodyItem>
#include <cnoid/OperableSceneBody>
#include <cnoid/Link>

using namespace cnoid;

void BalanceMarkers::setup(BodyItem* bodyItem)
{
    bodyItem_ = bodyItem;
    if(!bodyItem_ || !bodyItem_->sceneBody()){
        return;
    }

    root_ = new SgGroup;
    comMarker_  = new SphereMarker(0.045, Vector3f(1.0f, 0.1f, 0.1f), 0.15f);
    cogMarker_  = new SphereMarker(0.035, Vector3f(1.0f, 0.55f, 0.0f), 0.10f);
    copMarker_  = new SphereMarker(0.050, Vector3f(0.1f, 0.95f, 0.1f), 0.10f);
    comProjMarker_ = new SphereMarker(0.030, Vector3f(0.15f, 0.35f, 1.0f), 0.20f);

    root_->addChild(comMarker_);
    root_->addChild(cogMarker_);
    root_->addChild(copMarker_);
    root_->addChild(comProjMarker_);

    bodyItem_->sceneBody()->insertEffectGroup(root_);
    attached_ = true;
}

void BalanceMarkers::clear()
{
    if(attached_ && bodyItem_ && bodyItem_->sceneBody() && root_){
        bodyItem_->sceneBody()->removeEffectGroup(root_);
    }
    attached_ = false;
    root_ = nullptr;
    comMarker_ = nullptr;
    cogMarker_ = nullptr;
    copMarker_ = nullptr;
    comProjMarker_ = nullptr;
    bodyItem_ = nullptr;
}

Vector3 BalanceMarkers::computeCoP(Body* body, bool& valid,
                                   double& totalForce, int& numContacts)
{
    valid = false;
    totalForce = 0.0;
    numContacts = 0;
    Vector3 weighted = Vector3::Zero();
    double sumFz = 0.0;

    const int n = body->numLinks();
    for(int i = 0; i < n; ++i){
        Link* link = body->link(i);
        if(!link) continue;
        for(const Link::ContactPoint& cp : link->contactPoints()){
            ++numContacts;
            const Vector3& f = cp.force();
            const Vector3& n = cp.normal();
            totalForce += f.norm();

            // Normal contact force (floor reaction along contact normal)
            const double fn = std::max(0.0, f.dot(n));
            if(fn <= 0.05) continue;

            weighted += cp.position() * fn;
            sumFz += fn;
        }
    }

    if(sumFz < 1e-2){
        return Vector3::Zero();
    }
    valid = true;
    return weighted / sumFz;
}

BalanceMetrics BalanceMarkers::update(Body* body)
{
    BalanceMetrics m;
    if(!body){
        return m;
    }

    body->calcCenterOfMass();
    m.com = body->centerOfMass();
    m.cog = m.com;
    m.comXY = Vector3(m.com.x(), m.com.y(), 0.015);
    m.cop = computeCoP(body, m.copValid, m.totalContactForce, m.numContacts);

    if(attached_){
        comMarker_->setTranslation(m.com);
        cogMarker_->setTranslation(m.cog);
        comProjMarker_->setTranslation(m.comXY);
        if(m.copValid){
            copMarker_->setRadius(0.050);
            copMarker_->setTranslation(Vector3(m.cop.x(), m.cop.y(), 0.015));
        } else {
            copMarker_->setRadius(0.001);
            copMarker_->setTranslation(m.comXY);
        }
    }

    return m;
}
