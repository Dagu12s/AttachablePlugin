/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//#include "TouchPlugin.hh"
#include "AttacherContact.hh"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/ParentEntity.hh"

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"



/////////////////////////////////////////////////////////////////////////
#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <string>
#include <iostream>

//#include "/home/david/Attacher/src/linking_try/include/AttachableJoint.hh"
#include <ignition/gazebo/System.hh>
/////////////////////////////////////////////////////

using namespace ignition;
using namespace gazebo;
using namespace systems;

class attacher_contact::AttacherContactPrivate
{
  // Initialize the plugin
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Actual function that enables the plugin.
  /// \param[in] _value True to enable plugin.
  public: void Enable(const bool _value);

  /// \brief Process contact sensor data and determine if a touch event occurs
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm);

  /// \brief Add target entities. Called when new collisions are found
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \param[in] _entities List of potential entities to add to targetEntities.
  public: void AddTargetEntities(const EntityComponentManager &_ecm,
                                 const std::vector<Entity> &_entities);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Transport node to keep services alive
  transport::Node node;

  /// \brief Collision entities that have been designated as contact sensors.
  /// These will be checked against the targetEntities to establish whether this
  /// model is touching the targets
  public: std::vector<Entity> collisionEntities;

  /// \brief Name of target. Kept for debug printing
  public: std::string targetName;

  /// \brief Target collisions which this model should be touching.
  public: std::vector<Entity> targetEntities;

  /// \brief std::chrono::duration type used throught this plugin
  public: using DurationType = std::chrono::duration<double>;

  /// \brief Target time to continuously touch.
  public: DurationType targetTime{0};

  /// \brief Time when started touching.
  public: DurationType touchStart{0};

  /// \brief Namespace for transport topics.
  public: std::string ns;
// ({this->parentLinkEntity, this->childLinkEntity, "fixed"})
  /// \brief Publisher which publishes a message after touched for enough time
  public: std::optional<transport::Node::Publisher> touchedPub;

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid and the post update can run
  public: bool validConfig{false};

  /// \brief Whether the plugin is enabled.
  public: bool enabled{false};

  /// \brief Entity of attachment link in the parent model
  private: ignition::gazebo::Entity sensorLinkEntity{ignition::gazebo::kNullEntity};

  /// \brief Entity of attachment link in the child model
  private: ignition::gazebo::Entity targetLinkEntity{ignition::gazebo::kNullEntity};

  /// \brief topic with the links to check if they are touching
  public: std::string attachtopic; 

  public: void OnAttachRequest(const ignition::msgs::StringMsg &_msg);
 
  /// \brief Name of Parent Model
  public: std::string sensorModelName;

  /// \brief Name of Parent Model
  public: std::string sensorLinkName;

  /// \brief Name of target model
  public: std::string targetModelName;

  /// \brief Name of target Link
  public: std::string targetLinkName;

  /// \brief Name of attachment link in the child model

  public: std::string sensorLink;

  
  public: std::atomic<bool> attachRequested{false};

  private: ignition::gazebo::Entity sensorEntity{ignition::gazebo::kNullEntity};

};

//////////////////////////////////////////////////
void attacher_contact::AttacherContactPrivate::Load(const EntityComponentManager &_ecm,
                         const sdf::ElementPtr &_sdf)
{
  // Target time
  if (!_sdf->HasElement("time"))
  {
    ignerr << "Missing required parameter <time>" << std::endl;
    return;
  }

  this->targetTime = DurationType(_sdf->Get<double>("time"));

  // Topic time
  if (_sdf->HasElement("topic"))
  {
    this->attachtopic = _sdf->Get<std::string>("topic");
  }
  else
  {
    this->attachtopic = "/AttacherContact/contact";
  }
  //ignmsg << "loop"<< std::endl;
  this->node.Subscribe(
      this->attachtopic, &attacher_contact::AttacherContactPrivate::OnAttachRequest, this);

  ignmsg << "AttacherContact subscribing to messages on "
          << "[" << this->attachtopic << "]" << std::endl;
 
  this->validConfig = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
void attacher_contact::AttacherContactPrivate::Enable(const bool _value)
{

  if (_value)
  {
    this->touchedPub.reset();
    this->touchedPub = this->node.Advertise<msgs::Boolean>("AttacherContact/touched");

    this->touchStart = DurationType::zero();
    this->enabled = true;

    igndbg << "Started touch plugin [" << this->ns << "]" << std::endl;
  }
  else
  {
    this->touchedPub.reset();
    this->enabled = false;

    this->targetEntities.clear(); //= kNullEntity;
    this->collisionEntities.clear();
    igndbg << "Stopped touch plugin [" << this->ns << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
attacher_contact::AttacherContact::AttacherContact()
    : System(), dataPtr(std::make_unique<AttacherContactPrivate>())
{
}

//////////////////////////////////////////////////
void attacher_contact::AttacherContactPrivate::Update(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm)
{
  IGN_PROFILE("attacher_contact::AttacherContactPrivate::Update");
  
  if (this->enabled == false) {
    return;
  }

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (_info.paused)
    return;

  bool touching{false};
  // Iterate through all the target entities and check if there is a contact
  // between the target entity and this model


  for (const Entity colEntity : this->collisionEntities)
  {
    auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);

    // ignerr << "Hay contactos ??? "<< "\n" ;

    if (contacts)
    {     
      // Check if the contacts include one of the target entities.
      for (const auto &contact : contacts->Data().contact())
      {
        bool col1Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision1().id());
        bool col2Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision2().id());
        if (col1Target || col2Target)
        {
          touching = true;
        }
      }
    }
  }

  if (!touching)
  {
    if (this->touchStart != DurationType::zero())
    {
      igndbg << "Model [" << this->model.Name(_ecm)
             << "] not touching anything at [" << _info.simTime.count()
             << "]" << std::endl;
    }
    this->touchStart = DurationType::zero();
      
    msgs::Boolean msg;
    msg.set_data(false);
    this->touchedPub->Publish(msg);
    //this->Enable(false);
  
    
    return;
  }
  else
  // Start touch timer
  {
    if (this->touchStart == DurationType::zero())
    {
      this->touchStart =
        std::chrono::duration_cast<DurationType>(_info.simTime);

      igndbg << "Model [" << this->model.Name(_ecm) << "] started touching ["
        << this->targetName << "] at " << this->touchStart.count() << " s"
        << std::endl;
    }
  }

  // Check if it has been touched for long enough
  auto completed = (std::chrono::duration_cast<DurationType>(_info.simTime) -
      this->touchStart) > this->targetTime;

  // This is a single-use plugin. After touched, publish a message
  // and stop updating
  if (completed)
  {
    igndbg << "Model [" << this->model.Name(_ecm) << "] touched ["
      << this->targetName << "] exclusively for "
      << this->targetTime.count() << " s" << std::endl;

    {
      if (this->touchedPub.has_value())
      {
        msgs::Boolean msg;
        msg.set_data(true);
        this->touchedPub->Publish(msg);
      }
    }
  }
}

//////////////////////////////////////////////////
void attacher_contact::AttacherContact::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void attacher_contact::AttacherContact::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  IGN_PROFILE("attacher_contact::AttacherContact::PreUpdate");
  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);   
    this->dataPtr->initialized = true;
  }

  if ( this->dataPtr->attachRequested)
  {
    this->dataPtr->attachRequested = false;

    auto modelEntity = _ecm.EntityByComponents(components::Model(),components::Name(this->dataPtr->targetModelName));
    if (ignition::gazebo::kNullEntity != modelEntity)
    { 
      auto linkEntity = _ecm.EntityByComponents(components::Link(), components::ParentEntity(modelEntity),
          components::Name(this->dataPtr->targetLinkName));
      
      if (ignition::gazebo::kNullEntity != linkEntity)
      {
        auto linkCollisions = _ecm.ChildrenByComponents(linkEntity, components::Collision());
        for (const Entity entity : linkCollisions) 
        {
          if (ignition::gazebo::kNullEntity != entity)
          {
            this->dataPtr->targetEntities.push_back(entity);
          }
        }
        std::sort(this->dataPtr->targetEntities.begin(), this->dataPtr->targetEntities.end());
      }
    }

    // Create a list of collision entities that have been marked as contact
    // sensors in this model. These are collisions that have a ContactSensorData
    // component

    modelEntity = _ecm.EntityByComponents(components::Model(),components::Name(this->dataPtr->sensorModelName));
    if (ignition::gazebo::kNullEntity != modelEntity)
    {
      auto linkEntity = _ecm.EntityByComponents(components::Link(), components::ParentEntity(modelEntity),
          components::Name(this->dataPtr->sensorLinkName));
      
      if (ignition::gazebo::kNullEntity != linkEntity)
      {
        auto linkCollisions = _ecm.ChildrenByComponents(linkEntity, components::Collision());

        for (const Entity entity : linkCollisions) 
        {
          if ((ignition::gazebo::kNullEntity != entity) && 
              (_ecm.EntityHasComponentType(entity, components::ContactSensorData::typeId)))
          {
            this->dataPtr->collisionEntities.push_back(entity);
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void attacher_contact::AttacherContact::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  IGN_PROFILE("attacher_contact::AttacherContact::PostUpdate");
  if (this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}



void attacher_contact::AttacherContactPrivate::OnAttachRequest(const ignition::msgs::StringMsg &msg)
{
    
  
  ignmsg << "El mensaje enviado es: " << msg.data() << std::endl;

  // [Model][AttachableLink1][Model][AttachableLink2]
  //Now the Link must be nammed AttachableLink_Name or wont work

  std::string str = msg.data();

  if(this->enabled == true)
  {
    if (str.find("end") != -1)
    {
      this->Enable(false);
      ignmsg << "Disabled";
    }

  }
  else
  {
    unsigned first = str.find('[');

    if (first != -1)
    {
      str = &str[first];
      unsigned last = str.find(']');
      this->sensorModelName = str.substr(1,last-1);
      str = &str[last];

      first = str.find('[');
      str = &str[first];
      last = str.find(']');
      this->sensorLinkName = str.substr(1,last-1);
      str = &str[last];

      first = str.find('[');
      str = &str[first];
      last = str.find(']');
      this->targetModelName = str.substr(1,last-1);
      str = &str[last];

      first = str.find('[');
      str = &str[first];
      last = str.find(']');
      this->targetLinkName = str.substr(1,last-1);

      this->attachRequested = true;
      this->Enable(true);

    }
  }
}




IGNITION_ADD_PLUGIN(attacher_contact::AttacherContact,
                    ignition::gazebo::System,
                    attacher_contact::AttacherContact::ISystemConfigure,
                    attacher_contact::AttacherContact::ISystemPreUpdate,
                    attacher_contact::AttacherContact::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(attacher_contact::AttacherContact, "attacher_contact::AttacherContact")
