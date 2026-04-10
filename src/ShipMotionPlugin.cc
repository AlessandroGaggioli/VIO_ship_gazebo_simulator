//ShipMotionPlugin.cc

/*
This file implements the ShipMotionPlugin class, 
which is a Gazebo plugin that simulates the motion of a ship by controlling its roll, pitch, and heave joints.
The plugin reads parameters from an SDF file to define the motion characteristics and applies forces to the joints using a PID controller.
The plugin is designed to be used in a Gazebo simulation environment 
and can be configured through an SDF file that specifies the desired motion parameters and joint names.

some key features of the plugin include:
- Configurable motion parameters: The plugin allows users to specify the amplitude, frequency, and phase for roll, pitch, and heave motions through the SDF file.
- PID control: The plugin implements a simple PID controller to calculate the forces needed to achieve the desired joint positions and velocities based on the current state of the joints.
- Ramp-up functionality: To ensure a smooth start of the motion, the plugin includes a ramp-up mechanism that gradually increases the applied forces over a specified duration at the beginning of the motion.
- Integration with Gazebo: The plugin is designed to work seamlessly within the Gazebo simulation environment, utilizing its entity-component system to manage the ship model and its joints.

TO DO:
- Implement a feed-forward control component to improve the accuracy of the joint movements by anticipating the required forces based on the desired motion profile.
- Tune gains of the PID controller (Kp, Ki, Kd) to achieve better performance and stability of the ship's motion.
- Modify the motion of the ship to follow a more realistic wave pattern, potentially by using a combination of sinusoidal functions or by incorporating real wave data into the motion calculations.
*/

#include "ship_gazebo/ShipMotionPlugin.hh"         

GZ_ADD_PLUGIN(
    ship_gazebo::ShipMotionPlugin, 
    gz::sim::System, 
    gz::sim::ISystemConfigure, 
    gz::sim::ISystemPreUpdate 
)

GZ_ADD_PLUGIN_ALIAS(
    ship_gazebo::ShipMotionPlugin, 
    "ship_gazebo::ShipMotionPlugin"
)

namespace ship_gazebo {

    void ShipMotionPlugin::Configure( 
        const gz::sim::Entity &_entity, 
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/) //to avoid a warning (we don't use _eventMgr)
    {
        this->model = gz::sim::Model(_entity) ; //assign a entity to the model 
    
        if(!this->model.Valid(_ecm)) { //check if the entity is valid in the component manager (_ecm)
            gzerr << "[ShipMotionPlugin] Entity not valid\n" ; 
            return ; 
        }
        
        // lambda function: a function defined inside another function, that can capture variables from the enclosing scope.
        //define a lambda function to read a double parameter from the SDF file if it exists
        auto loadParam = [&](const std::string &name,double &param) {
            /* [&] to access local variables of the method by reference
            &name -> string of the double to search in the SDF
            &param -> member variable to update, passed by reference
            */
            if(_sdf->HasElement(name))
                param = _sdf->Get<double>(name) ; 
        };

        //now the function returns a gz::sim::Entity entity
        auto loadJoint = [&](const std::string &name) -> gz::sim::Entity {
            gz::sim::Entity joint = this->model.JointByName(_ecm,name) ; 
            if (joint == gz::sim::kNullEntity)
                gzwarn << "[ShipMotionPlugin]" << name << "not found\n" ; 
            //return the joint entity read from the SDF file
            return joint ; 
        } ; 
        
        // read roll parameters from the SDF file
        loadParam("roll_amplitude", this->rollAmplitude);
        loadParam("roll_frequency", this->rollFrequency);
        loadParam("roll_phase",     this->rollPhase);

        // read pitch parameters from the SDF file
        loadParam("pitch_amplitude", this->pitchAmplitude);
        loadParam("pitch_frequency", this->pitchFrequency);
        loadParam("pitch_phase",     this->pitchPhase);

        // read heave parameters from the SDF file
        loadParam("heave_amplitude", this->heaveAmplitude);
        loadParam("heave_frequency", this->heaveFrequency);
        loadParam("heave_phase",     this->heavePhase);

        // read max step size (dt) from the SDF file
        loadParam("max_step_size",this->dt) ; 

        //read joints from the SDF file
        this->rollJoint = loadJoint("roll_joint") ; 
        this->pitchJoint = loadJoint("pitch_joint") ; 
        this->heaveJoint = loadJoint("heave_joint") ; 

        gzmsg << "[ShipMotionPlugin] Plugin correctly initialized\n" ; 
    } //void ShipMotionPlugin::Configure

    void ShipMotionPlugin::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
    {
        //step 1 - pause
        if(_info.paused) //if the simulation is paused - joints remain blocked 
            return ; 
        
        //step 2 - read simTime in second
        double t = std::chrono::duration<double>(_info.simTime).count() ; //<double> to convert the object std::chrono::duration in seconds, count() to estract the numeric value
        
        //step 3  - motion law - pos(t) = A * sin(2 pi_greco frequence t + phase) 
      
        double rollPos = 0.0 ; 
        double pitchPos = 0.0 ; 
        double heavePos = 0.0 ; 

        double rollVel  = 0.0 ; 
        double pitchVel = 0.0 ; 
        double heaveVel = 0.0 ;

        // Start motion only after: delay elapsed + joints stable for a continuous hold time
        auto jointIsStable = [&](gz::sim::Entity joint) {
            if (joint == gz::sim::kNullEntity)
                return true;

            auto *posComp = _ecm.Component<gz::sim::components::JointPosition>(joint);
            auto *velComp = _ecm.Component<gz::sim::components::JointVelocity>(joint);

            // If state is not yet available, wait one more update
            if (!posComp || posComp->Data().empty() || !velComp || velComp->Data().empty())
                return false;

            const double pos = posComp->Data()[0];
            const double vel = velComp->Data()[0];

            return std::abs(pos) <= this->settlePosThreshold &&
                   std::abs(vel) <= this->settleVelThreshold;
        };

        // Check if all joints are stable before starting motion
        const bool allJointsStable =
            jointIsStable(this->rollJoint) &&
            jointIsStable(this->pitchJoint) &&
            jointIsStable(this->heaveJoint);

        if (!this->motionStarted && t >= this->spawnDelay) { // Check if we should start the stabilization process
            if (allJointsStable) { // If all joints are stable, start or continue the settle timer
                if (!this->settleTimerRunning) { // If the timer is not already running, start it
                    this->settleTimerRunning = true;
                    this->settleStartTime = t;
                }

                if ((t - this->settleStartTime) >= this->settleHoldTime) { // If joints have been stable for the required hold time, start the motion
                    this->motionStarted = true;
                    this->motionStartTime = t;
                    printf("[ShipMotionPlugin] Ship stabilized, starting motion\n");
                }
            } else { // If any joint is not stable, reset the settle timer
                this->settleTimerRunning = false;
            }
        }

        if(this->motionStarted) { 
            double t_motion = t - this->motionStartTime ; //time since motion started
            
            // --- START RAMP-UP ---

            // If we are in the first 5 seconds of motion, the factor rises from 0 to 1 linearly
            if (t_motion < this->rampDuration) {
                this->rampFactor = t_motion / this->rampDuration;
            }
            // --- END RAMP-UP ---

            // Apply the ramp factor to the amplitudes for a smooth start
            rollPos = this->rampFactor * this->rollAmplitude * std::sin(2.0*M_PI*this->rollFrequency*t_motion+this->rollPhase) ; 
            pitchPos = this->rampFactor * this->pitchAmplitude * std::sin(2.0*M_PI*this->pitchFrequency*t_motion+this->pitchPhase) ; 
            heavePos = this->rampFactor * this->heaveAmplitude * std::sin(2.0*M_PI*this->heaveFrequency*t_motion+this->heavePhase) ; 

            rollVel  = this->rampFactor * this->rollAmplitude  * 2.0*M_PI*this->rollFrequency * std::cos(2.0*M_PI*this->rollFrequency*t_motion  + this->rollPhase);
            pitchVel = this->rampFactor * this->pitchAmplitude * 2.0*M_PI*this->pitchFrequency * std::cos(2.0*M_PI*this->pitchFrequency*t_motion + this->pitchPhase);
            heaveVel = this->rampFactor * this->heaveAmplitude * 2.0*M_PI*this->heaveFrequency * std::cos(2.0*M_PI*this->heaveFrequency*t_motion + this->heavePhase);
        }
        
        // step 4 - Custom PD-Controller -- method to control a joint (force command given target position and velocity)
        auto sendForceCmd = [&](gz::sim::Entity joint, double targetPos, double targetVel) {
            if(joint == gz::sim::kNullEntity) 
                return;

            // 1. Read current position
            double currentPos = 0.0;
            auto* posComp = _ecm.Component<gz::sim::components::JointPosition>(joint); //read position
            
            if(posComp && !posComp->Data().empty()) //if exist: 
                currentPos = posComp->Data()[0]; //upgrade current position
            else 
                _ecm.CreateComponent(joint, gz::sim::components::JointPosition()); //if it doesn't exist - create it (normally at first iteration)

            // 2. Read current velocity
            double currentVel = 0.0;
            auto* velComp = _ecm.Component<gz::sim::components::JointVelocity>(joint); // read velocity
            
            if(velComp && !velComp->Data().empty()) 
                currentVel = velComp->Data()[0];
            else 
                _ecm.CreateComponent(joint, gz::sim::components::JointVelocity()); 

            // 3. PID controller
            
            double errorPos = targetPos - currentPos;
            double errorVel = targetVel - currentVel;

            double currentIntegral = 0.0 ; 

            if (joint == this->rollJoint) {
                this->rollIntegralError += errorPos * this->dt;
                // Anti-windup (using clamp)
                this->rollIntegralError = std::clamp(this->rollIntegralError, -1.0, 1.0); //std::clamp to anti-windup the integral term
                currentIntegral = this->rollIntegralError;
            } else if(joint == this->pitchJoint) {
                this->pitchIntegralError += errorPos * this->dt;
                this->pitchIntegralError = std::clamp(this->pitchIntegralError, -1.0, 1.0); //std::clamp to anti-windup the integral term
                currentIntegral = this->pitchIntegralError;
            }
            else if (joint == this->heaveJoint) {
                this->heaveIntegralError += errorPos * this->dt ; 
                this->heaveIntegralError = std::clamp(this->heaveIntegralError,-0.5,0.5) ;  //std::clamp to anti-windup the integral term
                currentIntegral = this->heaveIntegralError ; 
            }
            
            double forceCmd = (this->Kp * errorPos) + (this->Kd * errorVel) +(this->Ki * currentIntegral)   ;

            // 4. Torque 
            auto* forceComp = _ecm.Component<gz::sim::components::JointForceCmd>(joint); //read the force
            if(forceComp) 
                forceComp->Data() = {forceCmd};
            else _ecm.CreateComponent(joint, gz::sim::components::JointForceCmd({forceCmd}));
        };

        //apply force to the joints
        sendForceCmd(this->rollJoint, rollPos, rollVel); 
        sendForceCmd(this->pitchJoint, pitchPos, pitchVel);
        sendForceCmd(this->heaveJoint, heavePos, heaveVel);
    }
}//namespace ship_gazebo
