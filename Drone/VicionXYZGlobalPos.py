from __future__ import print_function
from vicon_dssdk import ViconDataStream
import time
client = ViconDataStream.RetimingClient()

try:
    client.Connect( "192.168.1.33:801" ) #! ethernet (2) on the vicon computer
                                      #! remember to connect to local router

    # Check the version
    print( 'Version', client.GetVersion() )

    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print( 'X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis )

    #client.SetMaximumPrediction( 10 )
    print( 'Maximum Prediction', client.MaximumPrediction() )
    
    
    
    while( True ):
        #sleep 1 milisecond
        #time.sleep(0.001)
        try:
            client.UpdateFrame()

            subjectNames = client.GetSubjectNames()
            for subjectName in subjectNames:
                 #print( subjectName )
                 segmentNames = client.GetSegmentNames( subjectName )
                 for segmentName in segmentNames:
                     segmentChildren = client.GetSegmentChildren( subjectName, segmentName )
                     for child in segmentChildren:
                         try:
                             print( child, 'has parent', client.GetSegmentParentName( subjectName, segmentName ) )
                         except ViconDataStream.DataStreamException as e:
                             print( 'Error getting parent segment', e )
                     print( segmentName, 'has global translation', client.GetSegmentGlobalTranslation( subjectName, segmentName ) ) 
        except ViconDataStream.DataStreamException as e:
            print( 'Handled data stream error', e )
except ViconDataStream.DataStreamException as e:
    print( 'Handled data stream error', e )