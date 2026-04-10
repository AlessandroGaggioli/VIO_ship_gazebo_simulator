//ShipMotionPlugin.hh

/*
This file is part of the Ship Gazebo Plugin, a plugin for the Gazebo simulator that simulates ship motion.
*/

#pragma once //to include only once time 

#include <gz/sim/System.hh> 
#include <gz/sim/Model.hh>  
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <sdf/sdf.hh>
#include <memory>     
#include <gz/plugin/Register.hh> 
#include <chrono>
#include <cmath>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>


namespace ship_gazebo {

    class ShipMotionPlugin: //heredity
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    
    public: 
        ShipMotionPlugin() = default ; //constructor
        ~ShipMotionPlugin() override = default ; //destroyer

        void Configure( //exec once at beginning 
            const gz::sim::Entity &_entity, //model entity (ID)
            const std::shared_ptr<const sdf::Element> &_sdf, //pointer to SDF file
            gz::sim::EntityComponentManager &_ecm, //simulation database
            gz::sim::EventManager &_eventMgr //simulation events manager 
        ) override; //override per sovrascrivere il metodo della classe padre 

        void PreUpdate( //exec at every step
            const gz::sim::UpdateInfo &_info, //object time info
            /*[_info.simTime = current simulation time, 
            _info.paused = true if simulation is paused, 
            _info.dt = timestep value] */
            gz::sim::EntityComponentManager &_ecm //simulation database
            
        ) override; //as before

    private:
        //ship model entity
        gz::sim::Model model{gz::sim::kNullEntity} ; 

        //three joints entity 
        gz::sim::Entity rollJoint{gz::sim::kNullEntity} ; 
        gz::sim::Entity pitchJoint{gz::sim::kNullEntity} ; 
        gz::sim::Entity heaveJoint{gz::sim::kNullEntity} ; 

        //roll parameters
        double rollAmplitude{0.0} ;//rad
        double rollFrequency{0.0} ;//Hz
        double rollPhase{0.0} ;//rad
        
        //pitch parameters
        double pitchAmplitude{0.0} ;//rad
        double pitchFrequency{0.0} ;//Hz
        double pitchPhase{0.0} ;//rad

        //heave parameters
        double heaveAmplitude{0.0} ;//rad
        double heaveFrequency{0.0} ;//Hz
        double heavePhase{0.0} ;//rad

        //PID 
        double rollIntegralError{0.0} ; 
        double pitchIntegralError{0.0} ; 
        double heaveIntegralError{0.0} ; 
        double Ki{10000} ;
        double Kp{1666666.6} ; 
        double Kd{158333.3} ; 
        double dt{0.0005} ; 

        // Stabilization gate before starting periodic motion
        double spawnDelay{1.0} ; //delay before starting motion, to allow joints to settle at initial position
        double settlePosThreshold{0.01} ; //position error threshold to consider the joint as "settled" at initial position
        double settleVelThreshold{0.01} ; //velocity threshold to consider the joint as "settled" at initial position
        double settleHoldTime{1.0} ; //time duration for which the joints must remain "settled" before starting motion
        bool settleTimerRunning{false} ; //flag to indicate if the settle timer is running
        bool motionStarted{false} ; //flag to indicate if the motion has started
        double settleStartTime{0.0} ; //time at which the settle timer started
        double motionStartTime{0.0} ; //time at which the motion started

        double rampDuration = 5.0; // Duration of the ramp in seconds
        double rampFactor = 1.0;   // default multiplier (100% of the force)
            
        
        
    }; //class ShipMotionPlugin

} //namespace ship_gazebo