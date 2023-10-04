#ifndef DETECTED_OBJECTS_POS_DISPLAY_H
#define DETECTED_OBJECTS_POS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
#include <tracking_msgs/PosDetections.h>
#include "object_display_common.h"
#endif

namespace tracking_rviz_plugin
{

    struct DetectedObjectVisual
    {
        std::shared_ptr<Ogre::SceneNode> sceneNode;

        std::shared_ptr<ObjectVisual> objectVisual;
        std::shared_ptr<TextNode> detectionIdText, classText, confidenceText, sensorText;
        std::shared_ptr<rviz::Arrow> orientationArrow;
        std::shared_ptr<CovarianceVisual> covarianceVisual;

        float classConfidence;
        bool hasValidOrientation;
        unsigned int detectionId;
    };

    class DetectedObjectsPosDisplay: public ObjectDisplayCommon<tracking_msgs::PosDetections>
    {
    Q_OBJECT
    public:
        DetectedObjectsPosDisplay() {};
        virtual ~DetectedObjectsPosDisplay();
        virtual void onInitialize();

    protected:
        virtual void reset();

        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private Q_SLOTS:
        void objectVisualTypeChanged();

        virtual void stylesChanged();

    private:
        // Function to handle an incoming ROS message.
        void processMessage(const tracking_msgs::PosDetections::ConstPtr& msg);
       
        // All currently active tracks, with unique track ID as map key
        vector<std::shared_ptr<DetectedObjectVisual> > m_previousDetections;

        // Properties
        rviz::BoolProperty* m_render_covariances_property;
        rviz::BoolProperty* m_render_detection_ids_property;
        rviz::BoolProperty* m_render_confidences_property;
        rviz::FloatProperty* m_low_confidence_threshold_property;
        rviz::FloatProperty* m_low_confidence_alpha_property;
        rviz::BoolProperty* m_render_orientations_property;
        rviz::BoolProperty* m_render_sensor_text_property;

        rviz::FloatProperty* m_text_spacing_property;
        rviz::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace tracking_rviz_plugin

#endif 