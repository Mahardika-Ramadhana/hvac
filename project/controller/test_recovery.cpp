#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <iostream>

using namespace cnoid;

int main(int argc, char** argv) {
    BodyLoader loader;
    BodyPtr body = loader.load(argc > 1 ? argv[1] : "../../../urdf/robot_esr_v2_rev.urdf");
    if(!body) { std::cout << "Failed to load URDF\n"; return 1; }

    // Place robot prone (face down). Root pitch = -90 deg
    Link* root = body->rootLink();
    root->p() = Vector3(0, 0, 0.15); // Approximate chest height
    Matrix3 R;
    R = AngleAxis(0, Vector3::UnitZ()) * AngleAxis(-1.5708, Vector3::UnitY()) * AngleAxis(0, Vector3::UnitX());
    root->R() = R;

    auto setQ = [&](const char* name, double q) {
        if(auto l = body->link(name)) l->q() = q;
    };

    auto checkLowestZ = [&]() {
        body->calcForwardKinematics();
        double minZ = 1000;
        std::string minName;
        for(int i=0; i<body->numLinks(); ++i) {
            Link* l = body->link(i);
            if(l->p().z() < minZ) {
                minZ = l->p().z();
                minName = l->name();
            }
        }
        std::cout << "Lowest point: " << minZ << " at link " << minName << std::endl;
        return minZ;
    };

    std::cout << "--- Stage 0 (Prone Fall) ---\n";
    // Zero all joints
    for(int i=0; i<body->numJoints(); ++i) body->joint(i)->q() = 0.0;
    checkLowestZ();

    std::cout << "--- Stage 1 (Kneeling) ---\n";
    setQ("right_knee_pitch_2", -1.5);
    setQ("left_knee_pitch_1", 1.5);
    setQ("right_knee_pitch_4", 2.0);
    setQ("left_knee_pitch_3", -2.0);
    setQ("right_arm_pitch_2", 1.5);
    setQ("left_arm_pitch_1", -1.5);
    setQ("right_arm_roll_4", -1.0);
    setQ("left_arm_roll_3", 1.0);
    checkLowestZ();

    std::cout << "--- Stage 2 (Squat) ---\n";
    setQ("right_knee_pitch_2", -0.5);
    setQ("left_knee_pitch_1", 0.5);
    setQ("right_knee_pitch_4", 2.0);
    setQ("left_knee_pitch_3", -2.0);
    setQ("right_knee_pitch_6", -1.0);
    setQ("left_knee_pitch_5", 1.0);
    setQ("right_arm_pitch_2", -0.5);
    setQ("left_arm_pitch_1", 0.5);
    checkLowestZ();

    return 0;
}
