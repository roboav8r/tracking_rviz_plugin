#ifndef Q_MOC_RUN
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/selection/selection_manager.h"
#include "tracked_objects_display.h"
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH


namespace tracking_rviz_plugin
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
void TrackedObjectsDisplay::onInitialize()
{
    ObjectDisplayCommon::onInitialize();

    QObject::connect(m_commonProperties->style, SIGNAL(changed()), this, SLOT(objectVisualTypeChanged()) );

    // m_occlusion_alpha_property = new rviz::FloatProperty( "Occlusion alpha", 0.3, "Alpha multiplier for occluded tracks", this, SLOT(stylesChanged()) );
    // m_occlusion_alpha_property->setMin( 0.0 );

    m_missed_alpha_property = new rviz::FloatProperty( "Missed alpha", 0.5, "Alpha multiplier for missed tracks", this, SLOT(stylesChanged()) );
    m_missed_alpha_property->setMin( 0.0 );

    m_history_length_property = new rviz::IntProperty( "History size", 100, "Number of prior track positions to display.", this, SLOT(stylesChanged()));
    m_history_length_property->setMin( 1 );
    m_history_length_property->setMax( 10000000 );

    m_tracking_frame_property   = new rviz::StringProperty( "Tracking frame", "odom", "Coordinate frame into which track history should be transformed. Usually the fixed frame of the tracker.", this, SLOT(stylesChanged()));

    m_delete_after_ncycles_property = new rviz::IntProperty( "Delete after no. cycles", 100, "After how many time steps to delete an old track that has not been seen again, including its history", this, SLOT(stylesChanged()));
    m_delete_after_ncycles_property->setMin( 0 );
    m_delete_after_ncycles_property->setMax( 10000000 );

    m_show_deleted_property   = new rviz::BoolProperty( "Show DELETED tracks", false, "Show tracks which have been marked as deleted", this, SLOT(stylesChanged()));
    // m_show_occluded_property  = new rviz::BoolProperty( "Show OCCLUDED tracks", true, "Show tracks which could not be matched to an detection due to sensor occlusion", this, SLOT(stylesChanged()));
    m_show_missed_property   = new rviz::BoolProperty( "Show MISSED tracks", true, "Show tracks which could not be matched to an detection but should be observable by the sensor", this, SLOT(stylesChanged()));
    m_show_matched_property   = new rviz::BoolProperty( "Show MATCHED tracks", true, "Show tracks which could be matched to an detection", this, SLOT(stylesChanged()));


    m_render_history_property           = new rviz::BoolProperty( "Render history", true, "Render prior track positions", this, SLOT(stylesChanged()));
    m_render_history_as_line_property   = new rviz::BoolProperty( "History as line", true, "Display history as line instead of dots", this, SLOT(stylesChanged()));
    m_render_object_property            = new rviz::BoolProperty( "Render object visual", true, "Render object visualization", this, SLOT(stylesChanged()));
    m_render_covariances_property       = new rviz::BoolProperty( "Render covariances", true, "Render track covariance ellipses", this, SLOT(stylesChanged()));
    m_render_velocities_property        = new rviz::BoolProperty( "Render velocities", true, "Render track velocity arrows", this, SLOT(stylesChanged()));
    m_render_ids_property               = new rviz::BoolProperty( "Render track IDs", true, "Render track IDs as text", this, SLOT(stylesChanged()));
    m_render_detection_ids_property     = new rviz::BoolProperty( "Render detection IDs", true, "Render IDs of the detection that a track was matched against, if any", this, SLOT(stylesChanged()));
    m_render_track_state_property       = new rviz::BoolProperty( "Render track state", true, "Render track state text", this, SLOT(stylesChanged()));

    m_history_min_point_distance_property = new rviz::FloatProperty( "Min. history point distance", 0.4, "Minimum distance between history points before a new one is placed", this, SLOT(stylesChanged()) );
    m_history_line_width_property = new rviz::FloatProperty( "Line width", 0.05, "Line width of history", m_render_history_as_line_property, SLOT(stylesChanged()), this );
    m_covariance_line_width_property = new rviz::FloatProperty( "Line width", 0.1, "Line width of covariance ellipses", m_render_covariances_property, SLOT(stylesChanged()), this );


    // TODO: Implement functionality
    //m_render_state_prediction_property  = new rviz::BoolProperty( "Render state prediction", true, "Render state prediction from Kalman filter", this, SLOT( updateRenderFlags() ));

    // Create a scene node for visualizing track history
    m_trackHistorySceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
}

TrackedObjectsDisplay::~TrackedObjectsDisplay()
{
    m_cachedTracks.clear();
}

// Clear the visuals by deleting their objects.
void TrackedObjectsDisplay::reset()
{
    ObjectDisplayCommon::reset();
    m_cachedTracks.clear();
}

void TrackedObjectsDisplay::update(float wall_dt, float ros_dt)
{
    // Move map scene node
    Ogre::Vector3 mapFramePosition; Ogre::Quaternion mapFrameOrientation;
    getContext()->getFrameManager()->getTransform(m_tracking_frame_property->getStdString(), ros::Time(0), mapFramePosition, mapFrameOrientation);
    Ogre::Matrix4 mapFrameTransform(mapFrameOrientation); mapFrameTransform.setTrans(mapFramePosition);
    m_trackHistorySceneNode->setPosition(mapFramePosition);
    m_trackHistorySceneNode->setOrientation(mapFrameOrientation);

    // Update position of deleted tracks (because they are not being updated by ROS messages any more)
    foreach(const track_map::value_type& entry, m_cachedTracks)
    {
        const boost::shared_ptr<TrackedObjectVisual>& trackedObjectVisual = entry.second;
        if(trackedObjectVisual->isDeleted) {
            Ogre::Matrix4 poseInCurrentFrame = mapFrameTransform * trackedObjectVisual->lastObservedPose;
            Ogre::Vector3 position = poseInCurrentFrame.getTrans(); Ogre::Quaternion orientation = poseInCurrentFrame.extractQuaternion();
            if(!position.isNaN() && !orientation.isNaN()) {
                trackedObjectVisual->sceneNode->setPosition(position);
                trackedObjectVisual->sceneNode->setOrientation(orientation);
            }
        }
        else {
            // Update animation etc.
            if(trackedObjectVisual->objectVisual) trackedObjectVisual->objectVisual->update(ros_dt);
        }
    }
}

/// Update all dynamically adjusted visualization properties (colors, font sizes etc.) of all currently tracked objects
void TrackedObjectsDisplay::stylesChanged()
{
    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) );

    // Update each track
    foreach(const track_map::value_type& entry, m_cachedTracks)
    {
        const track_id trackId = entry.first;
        const boost::shared_ptr<TrackedObjectVisual>& trackedObjectVisual = entry.second;

        // Update common styles to object visual, such as line width
        applyCommonStyles(trackedObjectVisual->objectVisual);

        // Update track visibility
        bool trackVisible = !isObjectHidden(trackId);

        if (trackedObjectVisual->isDeleted) trackVisible &= m_show_deleted_property->getBool();
        // else if(trackedObjectVisual->isOccluded) trackVisible &= m_show_occluded_property->getBool();
        else if(trackedObjectVisual->isMissed) trackVisible &= m_show_missed_property->getBool();
        else trackVisible &= m_show_matched_property->getBool();

        trackedObjectVisual->sceneNode->setVisible(trackVisible);
        trackedObjectVisual->historySceneNode->setVisible(trackVisible && !m_render_history_as_line_property->getBool());
        trackedObjectVisual->historyLineSceneNode->setVisible(trackVisible && m_render_history_as_line_property->getBool());

        // Get current track color
        Ogre::ColourValue trackColorWithFullAlpha = getColorFromId(trackId);
        Ogre::ColourValue trackColor = getColorFromId(trackId);
        trackColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
        // if(trackedObjectVisual->isOccluded) trackColor.a *= m_occlusion_alpha_property->getFloat(); // occlusion alpha
        if(trackedObjectVisual->isMissed) trackColor.a *= m_missed_alpha_property->getFloat(); // occlusion alpha

        // Update object color
        Ogre::ColourValue objectColor = trackColor;
        if(!m_render_object_property->getBool()) objectColor.a = 0.0;

        if(trackedObjectVisual->objectVisual) {
            trackedObjectVisual->objectVisual->setColor(objectColor);
        }

        // Update history size
        trackedObjectVisual->history.rset_capacity(m_history_length_property->getInt());

        // Update history color
        foreach(boost::shared_ptr<TrackedObjectHistoryEntry> historyEntry, trackedObjectVisual->history) {
            const double historyShapeDiameter = 0.1;
            Ogre::ColourValue historyColor = trackColorWithFullAlpha;
            historyColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
            // if(historyEntry->wasOccluded) historyColor.a *= m_occlusion_alpha_property->getFloat();
            if(isObjectHidden(trackId) || m_render_history_as_line_property->getBool()) historyColor.a = 0;

            if(historyEntry->shape) {
                historyEntry->shape->setColor(historyColor);
                historyEntry->shape->setScale(shapeQuaternion * Ogre::Vector3(historyShapeDiameter, historyShapeDiameter, 0.05));
            }
        }

        if(trackedObjectVisual->historyLine) { // history-as-line mode (as opposed to history-as-dots)
            Ogre::ColourValue historyColor = trackColorWithFullAlpha;
            historyColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
            if(isObjectHidden(trackId)) historyColor.a = 0;
            trackedObjectVisual->historyLine->setColor(historyColor.r, historyColor.g, historyColor.b, historyColor.a);
        }

        // Update text colors, font size and visibility
        const double objectHeight = trackedObjectVisual->objectVisual ? trackedObjectVisual->objectVisual->getHeight() : 0;
        Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : trackColor;
        fontColor.a = m_commonProperties->alpha->getFloat();

        trackedObjectVisual->detectionIdText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        // trackedObjectVisual->detectionIdText->setVisible(!trackedObjectVisual->isOccluded && m_render_detection_ids_property->getBool() && trackVisible);
        trackedObjectVisual->detectionIdText->setVisible(m_render_detection_ids_property->getBool() && trackVisible);
        trackedObjectVisual->detectionIdText->setColor(fontColor);
        trackedObjectVisual->detectionIdText->setPosition(Ogre::Vector3(0,0, -trackedObjectVisual->detectionIdText->getCharacterHeight()));

        trackedObjectVisual->stateText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        trackedObjectVisual->stateText->setVisible(m_render_track_state_property->getBool() && trackVisible);
        trackedObjectVisual->stateText->setColor(fontColor);
        trackedObjectVisual->stateText->setPosition(Ogre::Vector3(0,0, objectHeight + trackedObjectVisual->stateText->getCharacterHeight()));
            
        const double stateTextOffset = m_render_track_state_property->getBool() ? 1.2*trackedObjectVisual->stateText->getCharacterHeight() : 0;
        trackedObjectVisual->idText->setCharacterHeight(0.25 * m_commonProperties->font_scale->getFloat());
        trackedObjectVisual->idText->setVisible(m_render_ids_property->getBool() && trackVisible);
        trackedObjectVisual->idText->setColor(fontColor);
        trackedObjectVisual->idText->setPosition(Ogre::Vector3(0,0, objectHeight + trackedObjectVisual->idText->getCharacterHeight() + stateTextOffset));

        // Update velocity arrow color
        double arrowAlpha = m_render_velocities_property->getBool() ? trackColor.a : 0.0;
        if(trackedObjectVisual->hasZeroVelocity) arrowAlpha = 0.0;
        trackedObjectVisual->velocityArrow->setColor(Ogre::ColourValue(trackColor.r, trackColor.g, trackColor.b, arrowAlpha));

        // Set color of covariance visualization
        Ogre::ColourValue covarianceColor = trackColor;
        if(!m_render_covariances_property->getBool()) covarianceColor.a = 0.0;
        trackedObjectVisual->covarianceVisual->setColor(covarianceColor);
        trackedObjectVisual->covarianceVisual->setLineWidth(m_covariance_line_width_property->getFloat());
    }

    // Update global history visibility
    m_trackHistorySceneNode->setVisible(m_render_history_property->getBool());
}


// Set the rendering style (cylinders, meshes, ...) of tracked objects
void TrackedObjectsDisplay::objectVisualTypeChanged()
{
    foreach(const track_map::value_type& entry, m_cachedTracks)
    {
        const boost::shared_ptr<TrackedObjectVisual>& trackedObjectVisual = entry.second;
        trackedObjectVisual->objectVisual.reset();
        createObjectVisualIfRequired(trackedObjectVisual->sceneNode.get(), trackedObjectVisual->objectVisual);
    }
    stylesChanged();
}

// This is our callback to handle an incoming message.
void TrackedObjectsDisplay::processMessage(const tracking_msgs::TrackedObjects::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    // Transform from map/odometry frame into fixed frame, required to display track history if the fixed frame is not really "fixed" (e.g. base_link)
    Ogre::Vector3 mapFramePosition; Ogre::Quaternion mapFrameOrientation;
    getContext()->getFrameManager()->getTransform(m_tracking_frame_property->getStdString(), msg->header.stamp, mapFramePosition, mapFrameOrientation);
    Ogre::Matrix4 mapFrameTransform(mapFrameOrientation); mapFrameTransform.setTrans(mapFramePosition);

    // Transform required to fix orientation of any Cylinder shapes
    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) );
    stringstream ss;

    //
    // Iterate over all tracks in this message, see if we have a cached visual (then update it) or create a new one.
    //
    set<unsigned int> encounteredTrackIds;
    for (vector<tracking_msgs::TrackedObject>::const_iterator trackedObjectIt = msg->tracks.begin(); trackedObjectIt != msg->tracks.end(); ++trackedObjectIt)
    {
        boost::shared_ptr<TrackedObjectVisual> trackedObjectVisual;

        // See if we encountered this track ID before in this loop (means duplicate track ID)
        if (encounteredTrackIds.find(trackedObjectIt->track_id) != encounteredTrackIds.end()) {
            ROS_ERROR_STREAM("tracking_msgs::TrackedObjects contains duplicate track ID " << trackedObjectIt->track_id << "! Skipping duplicate track.");
            continue;
        }
        else {
            encounteredTrackIds.insert(trackedObjectIt->track_id);
        }

        // See if we have cached a track with this ID
        if (m_cachedTracks.find(trackedObjectIt->track_id) != m_cachedTracks.end()) {
            trackedObjectVisual = m_cachedTracks[trackedObjectIt->track_id];
        }
        else {
            // Create a new visual representation of the tracked object
            trackedObjectVisual = boost::shared_ptr<TrackedObjectVisual>(new TrackedObjectVisual);
            m_cachedTracks[trackedObjectIt->track_id] = trackedObjectVisual;

            // This scene node is the parent of all visualization elements for the tracked object
            trackedObjectVisual->sceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
            trackedObjectVisual->historySceneNode = boost::shared_ptr<Ogre::SceneNode>(m_trackHistorySceneNode->createChildSceneNode());
            trackedObjectVisual->historyLineSceneNode = boost::shared_ptr<Ogre::SceneNode>(m_trackHistorySceneNode->createChildSceneNode());
        }

        // These values need to be remembered for later use in stylesChanged()
        if(!trackedObjectIt->matched){
            trackedObjectVisual->isMissed = true;
        }
        else {
            trackedObjectVisual->isMissed = false;
        }

        // if(trackedObjectIt->is_occluded && !trackedObjectIt->is_matched){
        //     trackedObjectVisual->isOccluded = true;
        //     trackedObjectVisual->isMissed = false;
        // }
        // else if(!trackedObjectIt->is_occluded && !trackedObjectIt->is_matched){
        //     trackedObjectVisual->isOccluded = false;
        //     trackedObjectVisual->isMissed = true;
        // }
        // else {
        //     trackedObjectVisual->isOccluded = false;
        //     trackedObjectVisual->isMissed = false;
        // }

        trackedObjectVisual->isDeleted = false;
        trackedObjectVisual->numCyclesNotSeen = 0;

        Ogre::SceneNode* currentSceneNode = trackedObjectVisual->sceneNode.get();


        //
        // Object visualization
        //

        // Create new visual for the object itself, if needed
        boost::shared_ptr<ObjectVisual> &objectVisual = trackedObjectVisual->objectVisual;
        createObjectVisualIfRequired(currentSceneNode, objectVisual);

        const double objectHeight = objectVisual ? objectVisual->getHeight() : 0;
        const double halfObjectHeight = objectHeight / 2.0;


        //
        // Position of entire track
        //

        const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(trackedObjectIt->pose);
        setPoseOrientation(currentSceneNode, trackedObjectIt->pose, covXYZinTargetFrame, objectHeight);


        //
        // Track history
        //

        Ogre::Vector3 newHistoryEntryPosition = mapFrameTransform.inverse() * currentSceneNode->getPosition();

        const float MIN_HISTORY_ENTRY_DISTANCE = m_history_min_point_distance_property->getFloat(); // in meters
        if((trackedObjectVisual->positionOfLastHistoryEntry - newHistoryEntryPosition).length() > MIN_HISTORY_ENTRY_DISTANCE)
        {
            // General history
            boost::shared_ptr<TrackedObjectHistoryEntry> newHistoryEntry(new TrackedObjectHistoryEntry);
            newHistoryEntry->trackId = trackedObjectIt->track_id;
            newHistoryEntry->position = newHistoryEntryPosition; // used by history lines (below) even if no shape is set
            // newHistoryEntry->wasOccluded = trackedObjectIt->is_occluded;
            trackedObjectVisual->history.push_back(newHistoryEntry);

            // Always need to reset history line since history is like a queue, oldest element has to be removed but BillboardLine doesn't offer that functionality
            trackedObjectVisual->historyLine.reset(new rviz::BillboardLine(context_->getSceneManager(), trackedObjectVisual->historyLineSceneNode.get()) );

            if(m_render_history_as_line_property->getBool()) {
                // History lines
                if(trackedObjectVisual->history.size() >= 2) {
                    trackedObjectVisual->historyLine->setLineWidth(m_history_line_width_property->getFloat());
                    trackedObjectVisual->historyLine->setMaxPointsPerLine(trackedObjectVisual->history.size());

                    foreach(const boost::shared_ptr<TrackedObjectHistoryEntry>& historyEntry, trackedObjectVisual->history) {
                        historyEntry->shape.reset(); // remove existing dot shapes, if any, for better performance
                        trackedObjectVisual->historyLine->addPoint(historyEntry->position);
                    }
                }
            }
            else {
                // History dots
                newHistoryEntry->shape = boost::shared_ptr<rviz::Shape>(new rviz::Shape(rviz::Shape::Cylinder, context_->getSceneManager(), trackedObjectVisual->historySceneNode.get()));
                newHistoryEntry->shape->setPosition(newHistoryEntryPosition);
                newHistoryEntry->shape->setOrientation(shapeQuaternion);
            }

            trackedObjectVisual->positionOfLastHistoryEntry = newHistoryEntryPosition;
        }


        //
        // Texts
        //
        {
            if (!trackedObjectVisual->idText) {
                trackedObjectVisual->idText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
                trackedObjectVisual->stateText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
                trackedObjectVisual->detectionIdText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
            }

            // Detection ID
            ss.str(""); ss << "det " << trackedObjectIt->detection_id;
            trackedObjectVisual->detectionIdText->setCaption(ss.str());
           
            // Track state
            ss.str("");

            // if(trackedObjectIt->is_occluded && !trackedObjectIt->is_matched)
            //     ss << "OCCLUDED";
            // else if (!trackedObjectIt->is_occluded && !trackedObjectIt->is_matched)
            if (!trackedObjectIt->matched)
                ss << "MISSED";
            else
                ss << "MATCHED";

            trackedObjectVisual->stateText->setCaption(ss.str());
            
            // Track ID
            ss.str(""); ss << trackedObjectIt->track_id;
            trackedObjectVisual->idText->setCaption(ss.str());
        }

        //
        // Velocity arrows
        //
        if (!trackedObjectVisual->velocityArrow) {
            trackedObjectVisual->velocityArrow.reset(new rviz::Arrow(context_->getSceneManager(), currentSceneNode));
        }

        // Update velocity arrow
        {
            const Ogre::Vector3 velocityVector = getVelocityVector(trackedObjectIt->twist);

            if(velocityVector.isZeroLength() || velocityVector.length() > 100 || velocityVector.isNaN()) {
                if(!velocityVector.isZeroLength()) { // do not show warning for zero velocity
                    ROS_WARN("Track %lu has suspicious velocity (%.1f m/s), not showing velocity vector!", trackedObjectIt->track_id, velocityVector.length());
                }
            }
            else {
                const double objectRadius = 0.2;
                const Ogre::Vector3 velocityArrowAttachPoint(objectRadius, 0, halfObjectHeight); // relative to tracked object's scene node
                trackedObjectVisual->velocityArrow->setPosition(velocityArrowAttachPoint);
                trackedObjectVisual->velocityArrow->setOrientation(m_frameOrientation * currentSceneNode->getOrientation().Inverse() * Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(velocityVector));

                const double shaftLength = velocityVector.length(), shaftDiameter = 0.05, headLength = 0.2, headDiameter = 0.2;
                trackedObjectVisual->velocityArrow->set(shaftLength, shaftDiameter, headLength, headDiameter);
                trackedObjectVisual->hasZeroVelocity = velocityVector.length() < 0.05;
            }

            boost::shared_ptr<MeshObjectVisual> meshObjectVisual = boost::dynamic_pointer_cast<MeshObjectVisual>(objectVisual);
            if(meshObjectVisual) {
                meshObjectVisual->setWalkingSpeed(velocityVector.length());
            }
        }


        //
        // Covariance visualization
        //
        if(!trackedObjectVisual->covarianceVisual) {
            trackedObjectVisual->covarianceVisual.reset(new ProbabilityEllipseCovarianceVisual(context_->getSceneManager(), currentSceneNode));
        }

        // Update covariance ellipse
        {
            Ogre::Vector3 covarianceMean(0,0,0); // zero mean because parent node is already centered at pose mean
            trackedObjectVisual->covarianceVisual->setOrientation(currentSceneNode->getOrientation().Inverse());
            trackedObjectVisual->covarianceVisual->setMeanCovariance(covarianceMean, covXYZinTargetFrame);
        }

    } // end for loop over all tracked objects

    // Set all properties which can be dynamically in the GUI. This iterates over all tracks.
    stylesChanged();

    //
    // First hide, then delete old cached tracks which have not been seen for a while
    //
    set<unsigned int> trackIdsToDelete;
    for (map<unsigned int, boost::shared_ptr<TrackedObjectVisual> >::const_iterator cachedTrackIt = m_cachedTracks.begin(); cachedTrackIt != m_cachedTracks.end(); ++cachedTrackIt) {
        if (encounteredTrackIds.end() == encounteredTrackIds.find(cachedTrackIt->first)) {
            const boost::shared_ptr<TrackedObjectVisual>& trackedObjectVisual = cachedTrackIt->second;

            // Update state and visibility
            if(!trackedObjectVisual->isDeleted) {
                trackedObjectVisual->stateText->setCaption("DELETED");
                trackedObjectVisual->isDeleted = true;

                Ogre::Matrix4 lastObservedPose(trackedObjectVisual->sceneNode->getOrientation()); lastObservedPose.setTrans(trackedObjectVisual->sceneNode->getPosition());
                trackedObjectVisual->lastObservedPose = mapFrameTransform.inverse() * lastObservedPose;
            }

            if(!m_show_deleted_property->getBool()) trackedObjectVisual->sceneNode->setVisible(false);

            // Delete if too old
            if(++trackedObjectVisual->numCyclesNotSeen > m_delete_after_ncycles_property->getInt()) {
                trackIdsToDelete.insert(cachedTrackIt->first);
            }
        }
    }

    for (set<unsigned int>::const_iterator setIt = trackIdsToDelete.begin(); setIt != trackIdsToDelete.end(); ++setIt) {
        m_cachedTracks.erase(*setIt);
    }

    //
    // Update status (shown in property pane)
    //
    ss.str("");
    ss << msg->tracks.size() << " tracks received";
    setStatusStd(rviz::StatusProperty::Ok, "Tracks", ss.str());
}

} // end namespace tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tracking_rviz_plugin::TrackedObjectsDisplay, rviz::Display)
