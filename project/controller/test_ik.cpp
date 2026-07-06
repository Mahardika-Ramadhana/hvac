#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <iostream>

using namespace cnoid;

int main(int argc, char** argv) {
    BodyLoader loader;
    BodyPtr body = loader.load(argc > 1 ? argv[1] : "../../../urdf/robot_esr_v2_rev.urdf");
    if(!body) { std::cout << "Failed to load URDF" << std::endl; return 1; }
    
    Link* base = body->rootLink();
    // In our modified URDF, root_x_link is root. base_link is a child.
    Link* realBase = body->link("base_link");
    body->link("right_knee_pitch_2")->q() = -0.04;
    body->link("right_knee_pitch_4")->q() = 0.04;
    body->link("right_knee_pitch_6")->q() = 0.0; // modified
    body->link("right_knee_roll_6")->q() = 0.0;
    Link* rFoot = body->link("right_knee_roll_6");
    Link* lFoot = body->link("left_knee_roll_5");
    
    if(!realBase || !rFoot || !lFoot) { std::cout << "Missing links!" << std::endl; return 1; }
    
    // Set specific stand pose values to match applyStandPose
    for(int i=0; i<body->numJoints(); ++i) body->joint(i)->q() = 0.0;
    
    body->link("right_knee_pitch_2")->q() = -0.04;
    body->link("right_knee_pitch_4")->q() = 0.04;
    body->link("right_knee_pitch_6")->q() = 0.0;
    
    body->link("left_knee_pitch_1")->q() = 0.04;
    body->link("left_knee_pitch_3")->q() = -0.04;
    body->link("left_knee_pitch_5")->q() = 0.0;
    
    // Set specific stand pose values to match applyStandPose
    // We'll just leave it at 0 to see the zero pose.
    body->calcForwardKinematics();
    
    auto rPath = JointPath::getCustomPath(realBase, rFoot);
    auto lPath = JointPath::getCustomPath(realBase, lFoot);
    
    if(!rPath || !lPath) { std::cout << "Failed to create JointPath" << std::endl; return 1; }
    
    Vector3 rPos = rFoot->p() - realBase->p();
    Vector3 lPos = lFoot->p() - realBase->p();
    
    std::cout << "Initial Right Foot Pos relative to base_link: " << rPos.transpose() << std::endl;
    std::cout << "Initial Left Foot Pos relative to base_link: " << lPos.transpose() << std::endl;
    
    std::cout << "Right Foot Rotation relative to base_link: \n" << (realBase->R().transpose() * rFoot->R()) << std::endl;
    std::cout << "Left Foot Rotation relative to base_link: \n" << (realBase->R().transpose() * lFoot->R()) << std::endl;
    
    // Test IK
    cnoid::Isometry3 T_tgt;
    T_tgt.linear() = Matrix3::Identity();
    T_tgt.translation() = realBase->p() + Vector3(-0.0626, 0.0, -0.33);
    bool success = rPath->calcInverseKinematics(T_tgt);
    
    cnoid::Isometry3 T_tgt2;
    T_tgt2.linear() = rFoot->R();
    T_tgt2.translation() = realBase->p() + Vector3(-0.0626, 0.0, -0.33);
    bool success2 = rPath->calcInverseKinematics(T_tgt2);
    std::cout << "IK to -0.33 Z with initial rotation: " << (success2 ? "Success" : "Failed") << std::endl;
    
    return 0;
}
