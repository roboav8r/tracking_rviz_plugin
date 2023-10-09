#ifndef DETECTED_OBJECTS_DISPLAY_H
#define DETECTED_OBJECTS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
#include <tracking_msgs/DetectedObjects.h>
#include "object_display_common.h"
#endif

namespace tracking_rviz_plugin
{
    /// The visual of a tracked object
    struct DetectedObjectVisual
    {
        boost::shared_ptr<Ogre::SceneNode> sceneNode;

        boost::shared_ptr<ObjectVisual> objectVisual;
        boost::shared_ptr<TextNode> detectionIdText, classConfidenceText, modalityText, modDetText, classIdText, classStrText;
        // boost::shared_ptr<rviz::Arrow> orientationArrow;
        boost::shared_ptr<CovarianceVisual> covarianceVisual;

        float classConfidence;
        bool hasValidOrientation;
        unsigned int detectionId, classId;
        std::string classStr;
    };

    // The DetectedObjectsDisplay class itself just implements a circular buffer,
    // editable parameters, and Display subclass machinery.
    class DetectedObjectsDisplay: public ObjectDisplayCommon<tracking_msgs::DetectedObjects>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        DetectedObjectsDisplay() {};
        virtual ~DetectedObjectsDisplay();
       
        virtual void onInitialize();

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
        void processMessage(const tracking_msgs::DetectedObjects::ConstPtr& msg);
       
        // All currently active tracks, with unique track ID as map key
        vector<boost::shared_ptr<DetectedObjectVisual> > m_previousDetections;

        // Properties
        rviz::BoolProperty* m_render_covariances_property;
        rviz::BoolProperty* m_render_detection_ids_property;
        rviz::BoolProperty* m_render_class_ids_property;
        rviz::BoolProperty* m_render_class_str_property;
        rviz::BoolProperty* m_render_confidences_property;
        rviz::FloatProperty* m_low_confidence_threshold_property;
        rviz::FloatProperty* m_low_confidence_alpha_property;
        rviz::BoolProperty* m_render_orientations_property;
        rviz::BoolProperty* m_render_modality_text_property;

        rviz::FloatProperty* m_text_spacing_property;
        rviz::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace tracking_rviz_plugin

#endif // DETECTED_OBJECTS_DISPLAY_H
