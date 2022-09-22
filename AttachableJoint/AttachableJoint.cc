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

#include "AttachableJoint.hh"
//#include "/home/david/Attacher/src/linking_try/include/AttachableJoint.hh"
#include <ignition/gazebo/System.hh>

using namespace attachable_joint;

//using namespace ignition;
//using namespace gazebo;
//using namespace systems;

////////////////////////////////////////////////

/////////////////////////////////////////////////
void AttachableJoint::Configure(const ignition::gazebo::Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               ignition::gazebo::EntityComponentManager &_ecm,
               ignition::gazebo::EventManager &/*_eventMgr*/)
{
  ///Topics
  if (_sdf->HasElement("attachtopic")) 
  {
    this->attachtopic = _sdf->Get<std::string>("attachtopic");
  }
  else
  {
    this->attachtopic = "AttachableJoint";
    ignmsg << "'attachtopic' is 'AttachableJoint' by default.\n";
  }
  
  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  this->suppressParentWarning =
      _sdf->Get<bool>("suppress_parent_warning", this->suppressParentWarning)
          .first;
  this->validConfig = true;
}



////////////////////////////////////////////////// 
void AttachableJoint::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  //ignmsg << "loop"<< std::endl;
  IGN_PROFILE("AttachableJoint::PreUpdate"); 
  ignition::msgs::Int32 msg;

  if (this->not_initialized)
  {
    this->node.Subscribe(
        this->attachtopic, &AttachableJoint::OnAttachRequest, this);

    ignmsg << "AttachableJoint subscribing to messages on "
          << "[" << this->attachtopic << "]" << std::endl;

    this->not_initialized = false;

    ///////////////

    this->error_topic.reset();
    this->error_topic = this->node.Advertise<ignition::msgs::Int32>("AttachableJoint/error");

    ///////////////

  }

  if (this->validConfig && this->attachRequested)
  {
    
    bool createNewAttachableJoint = true;  
      
    for(auto& item: this->attachableJointList)
    {
      if(item.second == this->attachableJointName)
      {
        createNewAttachableJoint = false;
        break;
      }
    }
    if (createNewAttachableJoint == true)
    {
      ignition::gazebo::Entity pmodelEntity{ignition::gazebo::kNullEntity};
        
      pmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->parentModelName));

      if (ignition::gazebo::kNullEntity != pmodelEntity)
      {
        this->parentLinkEntity = _ecm.EntityByComponents(
            ignition::gazebo::components::Link(), ignition::gazebo::components::ParentEntity(pmodelEntity),
            ignition::gazebo::components::Name(this->parentLinkName));
      
        if (ignition::gazebo::kNullEntity != this->parentLinkEntity)
        {
          //Hacemos todo con el hijo
          ignition::gazebo::Entity cmodelEntity{ignition::gazebo::kNullEntity};
          
          cmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->childModelName));
          if (ignition::gazebo::kNullEntity != cmodelEntity)
          {
            this->childLinkEntity = _ecm.EntityByComponents(
                ignition::gazebo::components::Link(), ignition::gazebo::components::ParentEntity(cmodelEntity),
                ignition::gazebo::components::Name(this->childLinkName));
                
            if (ignition::gazebo::kNullEntity != this->childLinkEntity)
            {
              // Attach the models
              // We do this by creating a detachable joint entity.
              this->attachableJointEntity = _ecm.CreateEntity();
              _ecm.CreateComponent(
                  this->attachableJointEntity,
                  ignition::gazebo::components::DetachableJoint({this->parentLinkEntity,
                                              this->childLinkEntity, "fixed"})); 

              this->attachableJointList.push_back({this->attachableJointEntity, this->attachableJointName});
              this->initialized = true;
              this->attachRequested = false;
              msg.set_data(0);
            }
            else
            {
              this->attachRequested = false;
              ignwarn << "Child Link " << this->childLinkName
                      << " could not be found.\n";
              msg.set_data(1);
            }
          }
          else if (!this->suppressChildWarning)
          {
            this->attachRequested = false;
            ignwarn << "Child Model " << this->childModelName
                    << " could not be found.\n";
            msg.set_data(1);
          }
            //Hacemos todo con el hijo
        }
        else
        {
          this->attachRequested = false;
          ignwarn << "Parent Link " << this->parentLinkName
                  << " could not be found.\n";
          msg.set_data(1);
        }
      }
      else if (!this->suppressParentWarning)
      {
        ignwarn << "Parent Model " << this->parentModelName
                << " could not be found.\n"; //ignwarnignerr
        this->attachRequested = false;
        msg.set_data(1);
      }
    }
    else
    {
      this->attachRequested = false;
      msg.set_data(2);
    }
  }
  if (this->initialized)
  {
    if (this->detachRequested)
    {
      // Detach the models
      int i;
      msg.set_data(1);
      for(i=0;i<=this->attachableJointList.size();i++)      
      {
        if(this->attachableJointList[i].second == this->attachableJointName)
        {
          msg.set_data(0);
          // igndbg << "Removing entity: " << this->attachableJointList << std::endl;
          _ecm.RequestRemoveEntity(this->attachableJointList[i].first);

          this->attachableJointList.erase(this->attachableJointList.begin()+i);
          this->detachRequested = false;
          break;
        }
      }

    }
  }
  this->error_topic->Publish(msg);

}
 
//////////////////////////////////////////////////
void AttachableJoint::OnAttachRequest(const ignition::msgs::StringMsg &msg)
{
  ignmsg << "El mensaje enviado es: " << msg.data() << std::endl;
  
  // [parentModel][ParentLink][ChildModel][ChildLink]
  //Now the Link must be nammed AttachableLink_Name or wont work

  std::string str = msg.data();//"[box1][box_body][box2][box_body]";
  std::string request;

  unsigned first = str.find('[');
  str = &str[first];
  unsigned last = str.find(']');
  this->parentModelName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->parentLinkName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childModelName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childLinkName = str.substr(1,last-1);
  str = &str[last];

  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  request = str.substr(1,last-1);

  this->attachableJointName = this->parentModelName + "_" + this->parentLinkName + "_" + this->childModelName + "_" + this->childLinkName;
  
  if ("attach" == request)
  {
    this->attachRequested = true;

    ignmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
           << std::endl << "\n\n\n";

  }
  else if ("detach" == request)
  {
    if (false == this->not_initialized)
    {
      this->detachRequested = true;

      ignmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
            << std::endl << "\n\n\n";
    }
    else
    {
      ignmsg << "There is no AttachableJoint created yet";
    }
  }
  
  /*
  if ( (this->parentLinkName.find("AttachableLink") != -1) && (this->childLinkName.find("AttachableLink") != -1) ) {

    this->attachRequested = true;
    ignmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName 
           << std::endl;
  } 
  else {
      ignerr << "parent link or child link are not AttachableLinks"<< std::endl;
  }
  */

}


IGNITION_ADD_PLUGIN(attachable_joint::AttachableJoint,
                    ignition::gazebo::System,
                    attachable_joint::AttachableJoint::ISystemConfigure,
                    attachable_joint::AttachableJoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AttachableJoint,"attachable_joint::AtachableJoint")
