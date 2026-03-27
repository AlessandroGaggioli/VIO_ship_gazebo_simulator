//ShipMotionPlugin.cc

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
        this->model = gz::sim::Model(_entity) ; //assegna un entity a model 
    
        if(!this->model.Valid(_ecm)) { //controlla che l'entity esista nel database (_ecm)
            gzerr << "[ShipMotionPlugin] Entity not valid\n" ; 
            return ; 
        }
        
        // funziona lambda: funzione temporanea dentro un'altra funzione
        /* definiamo una funzione lambda che legge un parametro double dall' SDF se esiste*/
        auto loadParam = [&](const std::string &name,double &param) {
            /* [&] per accedere per riferimento alle variabili locali del metodo
            &name -> stringa del double da cercare nell' SDF
            &param -> variabile membro da aggiornare, passata per riferimento
            */
            if(_sdf->HasElement(name))
                param = _sdf->Get<double>(name) ; 
        };

        //ora la funzione restituisce un'entità gz::sim::Entity
        auto loadJoint = [&](const std::string &name) -> gz::sim::Entity {
            gz::sim::Entity joint = this->model.JointByName(_ecm,name) ; 
            if (joint == gz::sim::kNullEntity)
                gzwarn << "[ShipMotionPlugin]" << name << "not found\n" ; 
            //restituisco il valore del joint letto dal file SDF
            return joint ; 
        } ; 
        
        // lettura parametri roll
        loadParam("roll_amplitude", this->rollAmplitude);
        loadParam("roll_frequency", this->rollFrequency);
        loadParam("roll_phase",     this->rollPhase);

        // lettura parametri pitch
        loadParam("pitch_amplitude", this->pitchAmplitude);
        loadParam("pitch_frequency", this->pitchFrequency);
        loadParam("pitch_phase",     this->pitchPhase);

        // lettura parametri heave
        loadParam("heave_amplitude", this->heaveAmplitude);
        loadParam("heave_frequency", this->heaveFrequency);
        loadParam("heave_phase",     this->heavePhase);

        //load max_step_size 
        loadParam("max_step_size",this->dt) ; 

        //lettura joints 
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
        
       if(t>=3) {
            double t_motion = t - 2.0 ; 
            
            // --- INIZIO RAMPA ---
            double rampDuration = 5.0; // Durata della rampa in secondi
            double rampFactor = 1.0;   // Moltiplicatore di default (100% della forza)
            
            // Se siamo nei primi 5 secondi di movimento, il fattore sale da 0 a 1 linearmente
            if (t_motion < rampDuration) {
                rampFactor = t_motion / rampDuration;
            }
            // --- FINE RAMPA ---

            // Applichiamo il rampFactor alle ampiezze per un avvio morbido
            rollPos = rampFactor * this->rollAmplitude * std::sin(2.0*M_PI*this->rollFrequency*t_motion+this->rollPhase) ; 
            pitchPos = rampFactor * this->pitchAmplitude * std::sin(2.0*M_PI*this->pitchFrequency*t_motion+this->pitchPhase) ; 
            heavePos = rampFactor * this->heaveAmplitude * std::sin(2.0*M_PI*this->heaveFrequency*t_motion+this->heavePhase) ; 

            rollVel  = rampFactor * this->rollAmplitude  * 2.0*M_PI*this->rollFrequency * std::cos(2.0*M_PI*this->rollFrequency*t_motion  + this->rollPhase);
            pitchVel = rampFactor * this->pitchAmplitude * 2.0*M_PI*this->pitchFrequency * std::cos(2.0*M_PI*this->pitchFrequency*t_motion + this->pitchPhase);
            heaveVel = rampFactor * this->heaveAmplitude * 2.0*M_PI*this->heaveFrequency * std::cos(2.0*M_PI*this->heaveFrequency*t_motion + this->heavePhase);
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
                this->rollIntegralError = std::clamp(this->rollIntegralError, -1.0, 1.0);
                currentIntegral = this->rollIntegralError;
            } else if(joint == this->pitchJoint) {
                this->pitchIntegralError += errorPos * this->dt;
                this->pitchIntegralError = std::clamp(this->pitchIntegralError, -1.0, 1.0);
                currentIntegral = this->pitchIntegralError;
            }
            else if (joint == this->heaveJoint) {
                this->heaveIntegralError += errorPos * this->dt ; 
                this->heaveIntegralError = std::clamp(this->heaveIntegralError,-0.5,0.5) ; 
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
