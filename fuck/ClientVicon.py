from __future__ import print_function
import socket
from vicon_dssdk import ViconDataStream
import time

client = ViconDataStream.RetimingClient()

HOST = "192.168.60.251"  # Replace 'ubuntu_ip' with the IP address of your Ubuntu machine
PORT = 12345        # The port on which the server is listening

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))
    print("Connected to server")
    client.Connect( "192.168.1.33:801" ) #! ethernet (2) on the vicon computer
                                         #! remember to connect to local router

    # Check the version
    print( 'Version', client.GetVersion() )

    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print( 'X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis )

    #client.SetMaximumPrediction( 10 )
    print( 'Maximum Prediction', client.MaximumPrediction() )
    
    while True:
        data = ""  # Initialize the data variable
        dataSend = ""
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
                     print( segmentName, 'has global rotation( EulerXYZ )', client.GetSegmentGlobalRotationEulerXYZ( subjectName, segmentName ) )               
                     dataXYZ=str(client.GetSegmentGlobalTranslation( subjectName, segmentName ))+"\n"
                     dataEuler=str(client.GetSegmentGlobalRotationEulerXYZ( subjectName, segmentName ))+"\n"
                     #extract data to np.array, seperated by commas
                     x,y,z,_= dataXYZ.split(",")
                     roll,pitch,yaw,_= dataEuler.split(",")
                     dataSend = x+","+y+","+z+","+roll+","+pitch+","+yaw
                     dataSend = dataSend.replace("(","")
                     dataSend = dataSend.replace(")","")
                     dataSend = dataSend.replace(" ","")
                     print(dataSend)
                     time.sleep(0.01)
                     
        except ViconDataStream.DataStreamException as e:
            print( 'Handled data stream error', e )
        client_socket.sendall(dataSend.encode())

print("Connection closed")
