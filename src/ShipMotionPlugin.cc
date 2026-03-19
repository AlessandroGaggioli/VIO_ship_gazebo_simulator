//ShipMotionPlugin.cc

#include "ship_gazebo/ShipMotionPlugin.hh" 
#include <gz/plugin/Register.hh> 
#include <chrono>
#include <cmath>
#include <gz/sim/components/JointPositionReset.hh>

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
        double rollPos = this->rollAmplitude * std::sin(2.0*M_PI*this->rollFrequency*t+this->rollPhase) ; 
        double pitchPos = this->pitchAmplitude * std::sin(2.0*M_PI*this->pitchFrequency*t+this->pitchPhase) ; 
        double heavePos = this->heaveAmplitude * std::sin(2.0*M_PI*this->heaveFrequency*t+this->heavePhase) ; 

        //step 4 - set joints position
        auto setJointPos = [&](gz::sim::Entity joint,double pos) {
            //if joint is not configured, skip
            if(joint == gz::sim::kNullEntity)
                return ; 
            
            //JointPositionReset forza la posizione del joint bypassando la fisica [ JointPosition è di sola lettura ] 
            
            //cerca il component sul joint, restituisce il puntatore se il component esiste, nullptr se non esiste
            auto* resetComp = _ecm.Component<gz::sim::components::JointPositionReset>(joint);
            //create the component JointPositionReset if it doesn't exist (CreateComponent verrà chiamato solo la prima volta)
            if(resetComp)
                resetComp->Data() = {pos} ; 
            else
                _ecm.CreateComponent(joint,gz::sim::components::JointPositionReset({pos})); 
        } ; 

        setJointPos(this->rollJoint,rollPos) ; 
        setJointPos(this->pitchJoint,pitchPos) ; 
        setJointPos(this->heaveJoint,heavePos) ; 
    }
}//namespace ship_gazebo
