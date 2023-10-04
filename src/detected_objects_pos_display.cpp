#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/selection/selection_manager.h"

#include "detected_objects_pos_display.h"
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH

namespace tracking_rviz_plugin
{

void DetectedObjectsPosDisplay::onInitialize()
{
    ObjectDisplayCommon::onInitialize();

    QObject::connect(m_commonProperties->style, SIGNAL(changed()), this, SLOT(objectVisualTypeChanged()) );

    m_render_covariances_property       = new rviz::BoolProperty( "Render covariances", true, "Render track covariance ellipses", this, SLOT(stylesChanged()) );
    m_render_detection_ids_property     = new rviz::BoolProperty( "Render detection IDs", true, "Render IDs of the detection that a track was matched against, if any", this, SLOT(stylesChanged()));
    m_render_confidences_property       = new rviz::BoolProperty( "Render confidences", false, "Render detection confidences", this, SLOT(stylesChanged()));
    m_render_orientations_property      = new rviz::BoolProperty( "Render orientation arrows", true, "Render orientation arrows (only if orientation covariances are finite!)", this, SLOT(stylesChanged()));
    m_render_modality_text_property     = new rviz::BoolProperty( "Render modality text", false, "Render detection modality as text below detected person", this, SLOT(stylesChanged()));

    m_text_spacing_property = new rviz::FloatProperty( "Text spacing", 1.0, "Factor for vertical spacing betweent texts", this, SLOT(stylesChanged()), this );
    
    m_low_confidence_threshold_property = new rviz::FloatProperty( "Low-confidence threshold", 0.5, "Detection confidence below which alpha will be reduced", this, SLOT(stylesChanged()));
    m_low_confidence_alpha_property     = new rviz::FloatProperty( "Low-confidence alpha", 0.5, "Alpha multiplier for detections with confidence below low-confidence threshold", this, SLOT(stylesChanged()));

    m_covariance_line_width_property = new rviz::FloatProperty( "Line width", 0.1, "Line width of covariance ellipses", m_render_covariances_property, SLOT(stylesChanged()), this );
}

DetectedObjectsPosDisplay::~DetectedObjectsPosDisplay()
{
    m_previousDetections.clear();
    visuals_.clear()
}

// Clear the visuals by deleting their objects.
void DetectedObjectsPosDisplay::reset()
{
    ObjectDisplayCommon::reset();
    m_previousDetections.clear();
    visuals_.clear()
}

// Set the rendering style (cylinders, meshes, ...) of detected objects
void DetectedObjectsPosDisplay::objectVisualTypeChanged()
{
    foreach(shared_ptr<DetectedObjectVisual>& detectedObjectVisual, m_previousDetections)
    {
        detectedObjectVisual->objectVisual.reset();
        createObjectVisualIfRequired(detectedObjectVisual->sceneNode.get(), detectedObjectVisual->objectVisual);
    }
    stylesChanged();
}

// Update dynamically adjustable properties of all existing detections
void DetectedObjectsPosDisplay::stylesChanged()
{
    foreach(shared_ptr<DetectedObjectVisual>& detectedObjectVisual, m_previousDetections)
    {
        bool objectHidden = isObjectHidden(detectedObjectVisual->detectionId);

        // Update common styles to visual, such as line width
        applyCommonStyles(detectedObjectVisual->objectVisual);

        // Get current detection color
        Ogre::ColourValue detectionColor = getColorFromId(detectedObjectVisual->detectionId);
        detectionColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
        if(objectHidden) detectionColor.a = 0.0;
        if(detectedObjectVisual->confidence < m_low_confidence_threshold_property->getFloat()) detectionColor.a *= m_low_confidence_alpha_property->getFloat();

        if(detectedObjectVisual->objectVisual) {
            detectedObjectVisual->objectVisual->setColor(detectionColor);
        }

        // Update texts
        Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : detectionColor;
        fontColor.a = m_commonProperties->alpha->getFloat();
        if(objectHidden) fontColor.a = 0.0;
        
        float textOffset = 0.0f;
        detectedObjectVisual->detectionIdText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        detectedObjectVisual->detectionIdText->setPosition(Ogre::Vector3(0,0, -0.5*detectedObjectVisual->detectionIdText->getCharacterHeight() - textOffset));
        detectedObjectVisual->detectionIdText->setVisible(m_render_detection_ids_property->getBool());
        detectedObjectVisual->detectionIdText->setColor(fontColor);
        if(m_render_detection_ids_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedObjectVisual->detectionIdText->getCharacterHeight();

        detectedObjectVisual->modalityText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        detectedObjectVisual->modalityText->setPosition(Ogre::Vector3(textOffset, 0, -0.5*detectedObjectVisual->modalityText->getCharacterHeight() - textOffset));
        detectedObjectVisual->modalityText->setVisible(m_render_modality_text_property->getBool());
        detectedObjectVisual->modalityText->setColor(fontColor);
        if(m_render_modality_text_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedObjectVisual->modalityText->getCharacterHeight();

        detectedObjectVisual->confidenceText->setCharacterHeight(0.13 * m_commonProperties->font_scale->getFloat());
        detectedObjectVisual->confidenceText->setPosition(Ogre::Vector3(textOffset, 0, -0.5*detectedObjectVisual->confidenceText->getCharacterHeight() - textOffset));
        detectedObjectVisual->confidenceText->setVisible(m_render_confidences_property->getBool());
        detectedObjectVisual->confidenceText->setColor(fontColor);
        if(m_render_confidences_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedObjectVisual->confidenceText->getCharacterHeight();

        // Set color of covariance visualization
        Ogre::ColourValue covarianceColor = detectionColor;
        if(!m_render_covariances_property->getBool()) covarianceColor.a = 0.0;
        detectedObjectVisual->covarianceVisual->setColor(covarianceColor);
        detectedObjectVisual->covarianceVisual->setLineWidth(m_covariance_line_width_property->getFloat());

        // Update orientation arrow
        double arrowAlpha = m_render_orientations_property->getBool() && detectedObjectVisual->hasValidOrientation ? detectionColor.a : 0.0;
        detectedObjectVisual->orientationArrow->setColor(Ogre::ColourValue(detectionColor.r, detectionColor.g, detectionColor.b, arrowAlpha));
        const double shaftLength = 0.5, shaftDiameter = 0.05, headLength = 0.2, headDiameter = 0.2;
        detectedObjectVisual->orientationArrow->set(shaftLength, shaftDiameter, headLength, headDiameter);
    }
}

// This is our callback to handle an incoming message.
void DetectedObjectsPosDisplay::processMessage(const tracking_msgs::PosDetections::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ); // required to fix orientation of any Cylinder shapes
    stringstream ss;

    // Clear previous detections, this will also delete them from the scene graph
    m_previousDetections.clear();
    visuals_.clear();

    //
    // Iterate over all detections in this message and create a visual representation
    //
    for (vector<tracking_msgs::PosDetection>::const_iterator detectedObjectIt = msg->detections.begin(); detectedObjectIt != msg->detections.end(); ++detectedObjectIt)
    {
        shared_ptr<DetectedObjectVisual> detectedObjectVisual;

        // Create a new visual representation of the detected object
        detectedObjectVisual = shared_ptr<DetectedObjectVisual>(new DetectedObjectVisual);
        m_previousDetections.push_back(detectedObjectVisual);

        // This scene node is the parent of all visualization elements for the detected object
        detectedObjectVisual->sceneNode = shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
        detectedObjectVisual->detectionId = detectedObjectIt->detection_id;
        detectedObjectVisual->confidence = detectedObjectIt->class_confidence;
        Ogre::SceneNode* currentSceneNode = detectedObjectVisual->sceneNode.get();


        //
        // Object visualization
        //

        // Create new visual for the object itself, if needed
        shared_ptr<ObjectVisual> &objectVisual = detectedObjectVisual->objectVisual;
        createObjectVisualIfRequired(currentSceneNode, objectVisual);

        const double objectHeight = objectVisual ? objectVisual->getHeight() : 0;
        const double halfObjectHeight = objectHeight / 2.0;


        //
        // Position & visibility of entire detection
        //

        const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(detectedObjectIt->pose);
        setPoseOrientation(currentSceneNode, detectedObjectIt->pose, covXYZinTargetFrame, objectHeight);

        //
        // Texts
        //
        {
            // Detection ID
            if (!detectedObjectVisual->detectionIdText) {
                detectedObjectVisual->detectionIdText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
                detectedObjectVisual->detectionIdText->showOnTop();
            }

            ss.str(""); ss << "det " << detectedObjectIt->detection_id;
            detectedObjectVisual->detectionIdText->setCaption(ss.str());

            // Confidence value
            if (!detectedObjectVisual->confidenceText) {
                detectedObjectVisual->confidenceText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
            }

            ss.str(""); ss << fixed << setprecision(2) << detectedObjectIt->confidence;
            detectedObjectVisual->confidenceText->setCaption(ss.str());
            detectedObjectVisual->confidenceText->showOnTop();

            // Modality text
            if (!detectedObjectVisual->modalityText) {
                detectedObjectVisual->modalityText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
            }

            ss.str(""); ss << detectedObjectIt->modality;
            detectedObjectVisual->modalityText->setCaption(ss.str());
            detectedObjectVisual->modalityText->showOnTop();
        }

        //
        // Covariance visualization
        //
        if(!detectedObjectVisual->covarianceVisual) {
            detectedObjectVisual->covarianceVisual.reset(new ProbabilityEllipseCovarianceVisual(context_->getSceneManager(), currentSceneNode));
        }

        // Update covariance ellipse
        {
            Ogre::Vector3 covarianceMean(0,0,0); // zero mean because parent node is already centered at pose mean
            detectedObjectVisual->covarianceVisual->setOrientation(currentSceneNode->getOrientation().Inverse());
            detectedObjectVisual->covarianceVisual->setMeanCovariance(covarianceMean, covXYZinTargetFrame);
        }


        //
        // Orientation arrows
        //
        if (!detectedObjectVisual->orientationArrow) {
            detectedObjectVisual->orientationArrow.reset(new rviz::Arrow(context_->getSceneManager(), currentSceneNode));
        }

        // Update orientation arrow
        {
            const Ogre::Vector3 forwardVector(1,0,0);

            const double objectRadius = 0.2;
            const Ogre::Vector3 arrowAttachPoint(objectRadius, 0, halfObjectHeight); // relative to tracked object's scene node
            detectedObjectVisual->orientationArrow->setPosition(arrowAttachPoint);
            detectedObjectVisual->orientationArrow->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(forwardVector));
            detectedObjectVisual->hasValidOrientation = hasValidOrientation(detectedObjectIt->pose);
        }

    } // end for loop over all detected persons

    // Set all properties that can dynamically be adjusted in the GUI
    stylesChanged();

    //
    // Update status (shown in property pane)
    //
    ss.str("");
    ss << msg->detections.size() << " detections received";
    setStatusStd(rviz::StatusProperty::Ok, "Tracks", ss.str());
}

} // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tracking_rviz_plugin::DetectedObjectsPosDisplay, rviz::Display)