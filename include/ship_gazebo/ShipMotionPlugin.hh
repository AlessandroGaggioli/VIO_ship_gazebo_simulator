#pragma once //direttiva al compilatore, serve per include solo una volta questo file

#include <gz/sim/System.hh> //contiene le classi base del sistema
#include <gz/sim/Model.hh>  //contiene metodi utili del modello 
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <sdf/sdf.hh>
#include <memory>     


namespace ship_gazebo {

    class ShipMotionPlugin: //eredita
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    
    public: 
        ShipMotionPlugin() = default ; //costruttore
        ~ShipMotionPlugin() override = default ; //distruttore

        void Configure( //eseguito al primo step di simulazione
            const gz::sim::Entity &_entity, 
            /*entity del modello (passato per riferimento [&]) - identifica ship_corridor_dynamic */
            const std::shared_ptr<const sdf::Element> &_sdf, //rappresenta il tag XML dell' sdf 
            gz::sim::EntityComponentManager &_ecm, //database della simulazione
            gz::sim::EventManager &_eventMgr ///gestore degli eventi della simulazione
        ) override; //override per sovrascrivere il metodo della classe padre 

        void PreUpdate( //eseguito ad ogni step
            const gz::sim::UpdateInfo &_info, //struttura che contiene info sullo step corrente 
            /*[_info.simTime = tempo simulato corrente, 
            _info.paused = true se in pausa, 
            _info.dt = durata step] */
            gz::sim::EntityComponentManager &_ecm //database della simulazione, per scrivere le posizioni dei joint ad ogni step 
            
        ) override; //as before

    private:
        gz::sim::Model model{gz::sim::kNullEntity} ; //ship model

        //three joints entity - interi che identificano i joint inizializzati a kNullEntity (=0) finché Configure non li popola
        gz::sim::Entity rollJoint{gz::sim::kNullEntity} ; 
        gz::sim::Entity pitchJoint{gz::sim::kNullEntity} ; 
        gz::sim::Entity heaveJoint{gz::sim::kNullEntity} ; 

        //parametri roll -- inserire valori tra {} identificati come default se nell' SDF non si specifica nulla [come scrivere double rollAmplitude = 0.3]
        double rollAmplitude{0.3} ;//rad
        double rollFrequency{0.1} ;//Hz
        double rollPhase{0.0} ;//rad
        
        //parametri pitch 
        double pitchAmplitude{0.0} ;//rad
        double pitchFrequency{0.0} ;//Hz
        double pitchPhase{0.0} ;//rad

        //parametri heave
        double heaveAmplitude{0.0} ;//rad
        double heaveFrequency{0.0} ;//Hz
        double heavePhase{0.0} ;//rad
        
    }; //class ShipMotionPlugin

} //namespace ship_gazebo