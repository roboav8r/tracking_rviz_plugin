#ifndef TRACKED_OBJECTS_DISPLAY_H
#define TRACKED_OBJECTS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
// #include <spencer_tracking_msgs/TrackedPersons.h>
#include <tracking_msgs/TrackedObjects.h>
#include "object_display_common.h"
#endif

namespace tracking_rviz_plugin
{
    typedef unsigned int track_id;

    /// A single entry in the history of a tracked object.
    struct TrackedObjectHistoryEntry
    {
        Ogre::Vector3 position;
        boost::shared_ptr<rviz::Shape> shape;
        // bool wasOccluded;
        track_id trackId;
    };

    /// History of a tracked object.
    typedef circular_buffer<boost::shared_ptr<TrackedObjectHistoryEntry> > TrackedObjectHistory;

    /// The visual of a tracked object.
    struct TrackedObjectVisual
    {
        TrackedObjectHistory history;
        boost::shared_ptr<rviz::BillboardLine> historyLine;
        Ogre::Vector3 positionOfLastHistoryEntry;

        boost::shared_ptr<Ogre::SceneNode> sceneNode, historySceneNode, historyLineSceneNode;

        boost::shared_ptr<ObjectVisual> objectVisual;
        boost::shared_ptr<TextNode> detectionIdText, stateText, trackInfoText;
        boost::shared_ptr<rviz::Arrow> velocityArrow;
        boost::shared_ptr<CovarianceVisual> covarianceVisual;

        Ogre::Matrix4 lastObservedPose;

        // bool isOccluded, isDeleted, isMissed, hasZeroVelocity;
        bool isDeleted, isMissed, hasZeroVelocity;
        int numCyclesNotSeen;
    };

    // The TrackedObjectsDisplay class itself just implements a circular buffer,
    // editable parameters, and Display subclass machinery.
    class TrackedObjectsDisplay: public ObjectDisplayCommon<tracking_msgs::TrackedObjects>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        TrackedObjectsDisplay() {};
        virtual ~TrackedObjectsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.

        // Called after the constructors have run
        virtual void onInitialize();

        // Called periodically by the visualization manager
        virtual void update(float wall_dt, float ros_dt);

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private Q_SLOTS:
        void objectVisualTypeChanged();

        // Called whenever one of the properties in ObjectDisplayCommonProperties has been changed
        virtual void stylesChanged();

    private:
        // Function to handle an incoming ROS message.
        void processMessage(const tracking_msgs::TrackedObjects::ConstPtr& msg);
       
        // All currently active tracks, with unique track ID as map key
        typedef map<track_id, boost::shared_ptr<TrackedObjectVisual> > track_map;
        track_map m_cachedTracks;

        // Scene node for track history visualization
        boost::shared_ptr<Ogre::SceneNode> m_trackHistorySceneNode;
        std::string m_realFixedFrame;

        // User-editable property variables.
        rviz::FloatProperty* m_occlusion_alpha_property;
        rviz::FloatProperty* m_missed_alpha_property;
        rviz::StringProperty* m_tracking_frame_property;
        rviz::IntProperty*   m_history_length_property;
        rviz::IntProperty*   m_delete_after_ncycles_property;

        rviz::BoolProperty* m_show_deleted_property;
        // rviz::BoolProperty* m_show_occluded_property;
        rviz::BoolProperty* m_show_missed_property;
        rviz::BoolProperty* m_show_matched_property;
    
        rviz::BoolProperty* m_render_object_property;
        rviz::BoolProperty* m_render_history_property;
        rviz::BoolProperty* m_render_history_as_line_property;
        rviz::BoolProperty* m_render_covariances_property;
        rviz::BoolProperty* m_render_state_prediction_property;
        rviz::BoolProperty* m_render_velocities_property;
        rviz::BoolProperty* m_render_ids_property;
        rviz::BoolProperty* m_render_class_ids_property;
        rviz::BoolProperty* m_render_class_str_property;
        rviz::BoolProperty* m_render_class_conf_property;
        rviz::BoolProperty* m_render_detection_ids_property;
        rviz::BoolProperty* m_render_track_state_property;

        rviz::FloatProperty* m_history_line_width_property;
        rviz::FloatProperty* m_history_min_point_distance_property;
        rviz::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace tracking_rviz_plugin

#endif // TRACKED_OBJECTS_DISPLAY_H
