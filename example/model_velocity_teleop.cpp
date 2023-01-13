/**
 * @example model_velocity_teleop.cpp
 * Robot teleoperation using model-based velocity mirroring with force feedback
 * and additional algorithms.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Gripper.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>

namespace {
/** Operation modes for local and remote arm */
constexpr std::array<flexiv::Mode, 2> robotModes
    = {flexiv::MODE_TELEOP_LOCAL, flexiv::MODE_TELEOP_REMOTE};

/** Initial gripper width [m] */
constexpr double k_initGripperWidth = 0.05;

/** Default gripper moving velocity [m/s] */
constexpr double k_defaultGripperVel = 0.5;

/** Fail-safe counter limit */
constexpr size_t k_failSafeLimit = 10;

/** TCP force threshold to trigger the start of teleoperation [N] */
constexpr double k_tcpForceThreshold = 10.0;

}

/** Callback function for realtime periodic task */
void teleopPeriodicTask(
    std::array<std::shared_ptr<flexiv::Robot>, 2> robots, flexiv::Log& log)
{
    // Local periodic loop counter
    static size_t loopCounter = 0;

    // Local fail-safe counter
    static size_t failSafeCounter = 0;

    // Static data struct storing robot states
    static std::array<flexiv::RobotStates, 2> robotStates;

    try {
        // Check if all robots are operational
        for (auto& r : robots) {
            if (!r->isOperational()) {
                // Increment fail-safe counter
                failSafeCounter++;
            }
        }
        // If fail-safe counter reaches limit, stop all robots
        if (failSafeCounter > k_failSafeLimit) {
            for (auto& r : robots) {
                try {
                    r->stop();
                } catch (const flexiv::Exception& e) {
                    // Exception is expected when trying to stop robot that's
                    // not operational
                }
            }

            // Throw to trigger exit
            throw flexiv::ServerException(
                "Not all robots are operational, stopping and exiting ...");
        }
        // Reset fail-safe counter every 2s
        if (loopCounter % 2000 == 0) {
            failSafeCounter = 0;
        }

        // Read robot states for all robots
        for (size_t i = 0; i < robots.size(); i++) {
            robots[i]->getRobotStates(robotStates[i]);
        }

        // Send TCP pose of local arm to remote arm
        std::vector<double> localToRemoteM1 = robotStates[0].teleopAlgData.M1;
        double localToRemoteP1 = robotStates[0].teleopAlgData.P1;
        std::vector<double> localToRemoteP2 = robotStates[0].teleopAlgData.P2;

        robots[1]->commandToRemote(
            localToRemoteM1, localToRemoteP1, localToRemoteP2);

        // Send TCP wrench of remote arm to local arm
        std::vector<double> remoteToLocalWrench
            = robotStates[1].extWrenchInBase;
        double remoteToLocalP1 = robotStates[1].teleopAlgData.P1;
        std::vector<double> remoteToLocalP2 = robotStates[1].teleopAlgData.P2;
        std::vector<double> remoteToLocalP3 = robotStates[1].teleopAlgData.P3;

        robots[0]->commandToLocal(remoteToLocalWrench, remoteToLocalP1,
            remoteToLocalP2, remoteToLocalP3);

        // Increment loop counter
        loopCounter++;

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        exit(1);
    }
}

void gripperPeriodicTask(
    std::array<std::shared_ptr<flexiv::Gripper>, 2> grippers, flexiv::Log& log)
{
    // Static data struct storing gripper states
    static std::array<flexiv::GripperStates, 2> gripperStates;

    try {
        // Request gripper states for grippers
        for (size_t i = 0; i < grippers.size(); i++) {
            grippers[i]->getGripperStates(gripperStates[i]);
        }

        // Send current width of local gripper to remote gripper
        grippers[1]->move(gripperStates[0].width, k_defaultGripperVel);

        // If available, send current sensed force of remote gripper to local
        // gripper, flip sign to represent reactive force
        grippers[0]->grasp(-gripperStates[1].force);

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        exit(1);
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Mandatory arguments: [local arm IP] [remote arm IP] [local IP]" << std::endl;
    std::cout << "    local arm IP: address of the local arm" << std::endl;
    std::cout << "    remote arm IP: address of the remote arm" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: [--gripper]" << std::endl;
    std::cout << "    --gripper: enable gripper teleoperation" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 4
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the 2 robot servers
    std::array<std::string, 2> serverIPs = {argv[1], argv[2]};

    // IP of the workstation PC running this program
    std::string localIP = argv[3];

    // Check if user enabled gripper teleoperation
    bool isGripperEnabled = false;
    if (flexiv::utility::programArgsExist(argc, argv, "--gripper")) {
        isGripperEnabled = true;
        log.info("Gripper teleoperation enabled");
    }

    try {
        // RDK Initialization
        //=============================================================================
        // 2 robot interfaces
        std::array<std::shared_ptr<flexiv::Robot>, 2> robots;

        for (size_t i = 0; i < robots.size(); i++) {
            // Instantiate resources
            robots[i] = std::make_shared<flexiv::Robot>(serverIPs[i], localIP);

            // Clear fault on robot server if any
            if (robots[i]->isFault()) {
                log.warn("Fault occurred on robot server, trying to clear ...");
                // Try to clear the fault
                robots[i]->clearFault();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                // Check again
                if (robots[i]->isFault()) {
                    throw flexiv::ServerException(
                        "Fault cannot be cleared, exiting ...");
                }
                log.info("Fault on robot server is cleared");
            }

            // Enable all robot servers
            robots[i]->enable();
            log.info("[" + serverIPs[i] + "] Enabling robot ...");
        }

        // Run auto-recovery if needed
        bool isAnyRobotInRecovery = false;
        log.info("Checking for recovery state ...");
        std::this_thread::sleep_for(std::chrono::seconds(8));
        for (const auto& r : robots) {
            if (r->isRecoveryState()) {
                isAnyRobotInRecovery = true;
                r->startAutoRecovery();
            }
        }
        if (isAnyRobotInRecovery) {
            // Block forever, must reboot the robot and restart user program
            // after auto recovery is done
            log.warn("Please reboot robot when auto-recovery is done");
            while (true) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        // Wait for all robots to become operational
        for (size_t i = 0; i < robots.size(); i++) {
            int secondsWaited = 0;
            while (!robots[i]->isOperational()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (++secondsWaited == 10) {
                    log.warn("[" + serverIPs[i] + 
                    "] Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
                }
            }
            log.info("[" + serverIPs[i] + "] Robot is now operational");
        }

        // Initialize gripper control
        //=============================================================================
        // 2 gripper interfaces
        std::array<std::shared_ptr<flexiv::Gripper>, 2> grippers;
        if (isGripperEnabled) {
            for (size_t i = 0; i < grippers.size(); i++) {
                grippers[i]
                    = std::make_shared<flexiv::Gripper>(*(robots[i].get()));
                log.info("[" + serverIPs[i] + "] Initializing gripper ...");

                // Check maxWidth to determine if the gripper has been fully
                // initialized
                flexiv::GripperStates gs;
                while (gs.maxWidth < std::numeric_limits<double>::epsilon()) {
                    grippers[i]->getGripperStates(gs);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                // After gripper is ready, go to initial position
                log.info("[" + serverIPs[i]
                         + "] Gripper moving to initial position ...");
                grippers[i]->move(k_initGripperWidth, k_defaultGripperVel);
            }
        }

        // Reset all arms to home posture
        //=============================================================================
        // Set control mode for all robots after all are operational
        for (auto& r : robots) {
            r->setMode(flexiv::MODE_PRIMITIVE_EXECUTION);
            // Wait for mode to be switched
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (r->getMode() != flexiv::MODE_PRIMITIVE_EXECUTION);
        }

        for (auto& r : robots) {
            r->executePrimitive("Home()");
        }

        // Wait for the primitive to finish
        for (auto& r : robots) {
            while (r->isBusy()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        // Calibrate force/torque sensors
        //=============================================================================
        log.warn(
            "Calibrating force/torque sensors, please do not touch the robots");
        for (auto& r : robots) {
            r->executePrimitive("CaliForceSensor()");
        }
        // Wait for the primitive to finish
        for (auto& r : robots) {
            while (r->isBusy()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        log.info("Calibration complete");

        // Sync poses
        //=============================================================================
        // Create data struct for storing robot states
        std::array<flexiv::RobotStates, 2> robotStates;

        // Get current robot states data
        for (size_t i = 0; i < robots.size(); i++) {
            robots[i]->getRobotStates(robotStates[i]);
        }

        // Move both arm to the same position as local arm, with initial
        // orientation
        std::vector<double> posVec = {robotStates[0].tcpPose[0],
            robotStates[0].tcpPose[1], robotStates[0].tcpPose[2]};
        std::string posStr = flexiv::utility::vec2Str(posVec);
        for (auto& r : robots) {
            r->executePrimitive("MoveL(target=" + posStr
                                + "180 0 180 WORLD WORLD_ORIGIN, maxVel=0.3)");
        }

        // Wait for the primitive to finish
        for (auto& r : robots) {
            while (r->isStopped()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        // Wait for start signal
        //=============================================================================
        // Wait for the operator to take handle on the local arm
        log.info("Ready to teleoperate");
        Eigen::Vector3d tcpForce = Eigen::Vector3d::Zero();
        do {
            robots[0]->getRobotStates(robotStates[0]);
            tcpForce = {robotStates[0].extWrenchInBase[0],
                robotStates[0].extWrenchInBase[1],
                robotStates[0].extWrenchInBase[2]};
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (tcpForce.norm() < k_tcpForceThreshold);

        // Set new operation modes for all arms after they've reached ready pose
        for (size_t i = 0; i < robots.size(); i++) {
            robots[i]->setMode(robotModes[i]);

            // Wait for the mode to be switched
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robots[i]->getMode() != robotModes[i]);
        }

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Teleop control @ 1kHz, highest priority
        scheduler.addTask(std::bind(teleopPeriodicTask, robots, std::ref(log)),
            "Teleop periodic", 1, scheduler.maxPriority());

        // Gripper control @ 50Hz
        if (isGripperEnabled) {
            scheduler.addTask(
                std::bind(gripperPeriodicTask, grippers, std::ref(log)),
                "Gripper periodic", 20, scheduler.maxPriority() - 10);
        }

        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
