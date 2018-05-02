using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;
using WindowsPreview.Kinect;
using Windows.UI.Xaml.Media.Imaging;
using System.ComponentModel;
using Windows.Storage.Streams;
using System.Runtime.InteropServices;
using Windows.Storage;
using System.Threading.Tasks;
using System.Threading;

namespace Kinect2Sample
{
    public enum DisplayFrameType
    {
        Infrared,
        Color,
        Depth,
        BodyMask,
        BodyJoints
    }
    public sealed partial class MainPage : Page, INotifyPropertyChanged

    {
     
        private double[] features = new double[3];
        private ulong[] tracked = new ulong[6] {6,6,6,6,6,6};
        private List<int> order = new List<int>();
        private int body_num;

        private Dictionary<string, float> Features = new Dictionary<string, float>();

        //list of the features learned, to be used for the logging
        List<string> featuresList = new List<string>(new string[] { "HeadBackward", "HeadBentForward", "HeadOnHand_Left","HeadOnHand_Right",
                "HandOnHead_Left", "HandOnHead_Right","SpineForward","SpineBackward","ShouldersForward","ShouldersRaised",
                "ArmsAtTrunk","ArmsRaisedShoulder","HandsOnKnees","CrossedArms","ArmsRaisedUp","ArmsExtendedDown","HandsBehindHead","HandOnNeck_Left","HandOnNeck_Right" });

        SemaphoreSlim sem = new SemaphoreSlim(1);  //semaphore pr l'ecriture
        DateTime CurrentDate = DateTime.Now; //reference for the timestamps

        private const DisplayFrameType DEFAULT_DISPLAYFRAMETYPE = DisplayFrameType.Infrared;
        private FrameDescription currentFrameDescription;
        private DisplayFrameType currentDisplayFrameType;
        private MultiSourceFrameReader multiSourceFrameReader = null;
        private CoordinateMapper coordinateMapper = null;
        private BodiesManager bodiesManager = null;


        private KinectSensor kinectSensor = null;
        private string statusText = null;
        private WriteableBitmap bitmap = null;

        // Size of the RGB pixel in the bitmap
        private const int BytesPerPixel = 4;


        //Infrared Frame
        private InfraredFrameReader infraredFrameReader = null;
        private ushort[] infraredFrameData = null;
        private byte[] infraredPixels = null;

        //Depth Frame
        private ushort[] depthFrameData = null;
        private byte[] depthPixels = null;

        //BodyMask Frames
        private DepthSpacePoint[] colorMappedToDepthPoints = null;

        //Body Joints are drawn here
        private Canvas drawingCanvas;


        /// ** IR Variables **

        // The highest value that can be returned in the InfraredFrame.
        // It is cast to a float for readability in the visualization code.
        private const float InfraredSourceValueMaximum = (float)ushort.MaxValue;


        // Used to set the lower limit, post processing, of the infrared data that we will render.
        // Increasing or decreasing this value sets a brightness "wall" either closer or further away.
        private const float InfraredOutputValueMinimum = 0.01f;


        // The upper limit, post processing, of the infrared data that will render.
        private const float InfraredOutputValueMaximum = 1.0f;


        // The InfraredSceneValueAverage value specifies the average infrared value of the scene. 
        // This value was selected by analyzing the average pixel intensity for a given scene.
        // This could be calculated at runtime to handle different IR conditions of a scene (outside vs inside).
        private const float InfraredSceneValueAverage = 0.08f;


        // The InfraredSceneStandardDeviations value specifies the number of standard deviations to apply to InfraredSceneValueAverage.
        // This value was selected by analyzing data from a given scene.
        // This could be calculated at runtime to handle different IR conditions of a scene (outside vs inside).
        private const float InfraredSceneStandardDeviations = 3.0f;


        /// <summary> List of gesture detectors, 
        ///there will be one detector created for each potential body
        /// (max of 6) </summary>
        private List<GestureDetector> gestureDetectorList = null;

        public event PropertyChangedEventHandler PropertyChanged;
        public string StatusText
        {
            get { return this.statusText; }
            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new
                 PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        public FrameDescription CurrentFrameDescription
        {
            get { return this.currentFrameDescription; }
            set
            {
                if (this.currentFrameDescription != value)
                {
                    this.currentFrameDescription = value;
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new
         PropertyChangedEventArgs("CurrentFrameDescription"));
                    }
                }
            }
        }

        private void ShowBodyJoints(BodyFrame bodyFrame)
        {
            Body[] bodies = new Body[this.kinectSensor.BodyFrameSource.BodyCount];
            bool dataReceived = false;
            if (bodyFrame != null)
            {
                bodyFrame.GetAndRefreshBodyData(bodies);
                dataReceived = true;
            }

            if (dataReceived)
            {
                this.bodiesManager.UpdateBodiesAndEdges(bodies);
            }
        }

        unsafe private void ShowMappedBodyFrame(int depthWidth, int depthHeight, IBuffer bodyIndexFrameData, IBufferByteAccess bodyIndexByteAccess)
        {
            bodyIndexByteAccess = (IBufferByteAccess)bodyIndexFrameData;
            byte* bodyIndexBytes = null;
            bodyIndexByteAccess.Buffer(out bodyIndexBytes);

            fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = this.colorMappedToDepthPoints)
            {
                IBufferByteAccess bitmapBackBufferByteAccess = (IBufferByteAccess)this.bitmap.PixelBuffer;

                byte* bitmapBackBufferBytes = null;
                bitmapBackBufferByteAccess.Buffer(out bitmapBackBufferBytes);

                // Treat the color data as 4-byte pixels
                uint* bitmapPixelsPointer = (uint*)bitmapBackBufferBytes;

                // Loop over each row and column of the color image Zero out any pixels that don't correspond to a body index
                int colorMappedLength = this.colorMappedToDepthPoints.Length;
                for (int colorIndex = 0; colorIndex < colorMappedLength; ++colorIndex)
                {
                    float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                    float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                    // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                    if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                        !float.IsNegativeInfinity(colorMappedToDepthY))
                    {
                        // Make sure the depth pixel maps to a valid point in color space
                        int depthX = (int)(colorMappedToDepthX + 0.5f);
                        int depthY = (int)(colorMappedToDepthY + 0.5f);

                        // If the point is not valid, there is no body index there.
                        if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                        {
                            int depthIndex = (depthY * depthWidth) + depthX;

                            // If we are tracking a body for the current pixel, do not zero out the pixel
                            if (bodyIndexBytes[depthIndex] != 0xff)
                            {
                                // this bodyIndexByte is good and is a body, loop again.
                                continue;
                            }
                        }
                    }
                    // this pixel does not correspond to a body so make it black and transparent
                    bitmapPixelsPointer[colorIndex] = 0;
                }
            }

            this.bitmap.Invalidate();
            FrameDisplayImage.Source = this.bitmap;
        }

        private void ShowInfraredFrame(InfraredFrame infraredFrame)
        {
            bool infraredFrameProcessed = false;

            if (infraredFrame != null)
            {
                FrameDescription infraredFrameDescription =
                infraredFrame.FrameDescription;

                // verify data and write the new infrared frame data to the display bitmap
                if (((infraredFrameDescription.Width * infraredFrameDescription.Height)
                == this.infraredFrameData.Length) &&
                    (infraredFrameDescription.Width == this.bitmap.PixelWidth) &&
                (infraredFrameDescription.Height == this.bitmap.PixelHeight))
                {
                    // Copy the pixel data from the image to a temporary array
                    infraredFrame.CopyFrameDataToArray(this.infraredFrameData);

                    infraredFrameProcessed = true;
                }
            }

            // we got a frame, convert and render
            if (infraredFrameProcessed)
            {
                this.ConvertInfraredDataToPixels();
                this.RenderPixelArray(this.infraredPixels);
            }
        }

        private void ShowColorFrame(ColorFrame colorFrame)
        {
            bool colorFrameProcessed = false;

            if (colorFrame != null)
            {
                FrameDescription colorFrameDescription =
                    colorFrame.FrameDescription;

                // verify data and write the new color frame data to 
                // the Writeable bitmap
                if ((colorFrameDescription.Width ==
                    this.bitmap.PixelWidth) &&
             (colorFrameDescription.Height == this.bitmap.PixelHeight))
                {
                    if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                    {
                        colorFrame.CopyRawFrameDataToBuffer(
                            this.bitmap.PixelBuffer);
                    }
                    else
                    {
                        colorFrame.CopyConvertedFrameDataToBuffer(
                            this.bitmap.PixelBuffer,
                 ColorImageFormat.Bgra);
                    }

                    colorFrameProcessed = true;
                }
            }

            if (colorFrameProcessed)
            {
                this.bitmap.Invalidate();
                FrameDisplayImage.Source = this.bitmap;
            }
        }


        private void ConvertInfraredDataToPixels()
        {
            // Convert the infrared to RGB
            int colorPixelIndex = 0;
            for (int i = 0; i < this.infraredFrameData.Length; ++i)
            {
                // normalize the incoming infrared data (ushort) to a float ranging from InfraredOutputValueMinimum to InfraredOutputValueMaximum] by
                // 1. dividing the incoming value by the source maximum value
                float intensityRatio = (float)this.infraredFrameData[i] / InfraredSourceValueMaximum;

                // 2. dividing by the 
                // (average scene value * standard deviations)
                intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;

                // 3. limiting the value to InfraredOutputValueMaximum
                intensityRatio = Math.Min(InfraredOutputValueMaximum, intensityRatio);

                // 4. limiting the lower value InfraredOutputValueMinimum
                intensityRatio = Math.Max(InfraredOutputValueMinimum, intensityRatio);

                // 5. converting the normalized value to a byte and using 
                // the result as the RGB components required by the image
                byte intensity = (byte)(intensityRatio * 255.0f);
                this.infraredPixels[colorPixelIndex++] = intensity; //Blue
                this.infraredPixels[colorPixelIndex++] = intensity; //Green
                this.infraredPixels[colorPixelIndex++] = intensity; //Red
                this.infraredPixels[colorPixelIndex++] = 255;       //Alpha           
            }
        }


        private void ShowDepthFrame(DepthFrame depthFrame)
        {
            bool depthFrameProcessed = false;
            ushort minDepth = 0;
            ushort maxDepth = 0;

            if (depthFrame != null)
            {
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                // verify data and write the new infrared frame data
                // to the display bitmap
                if (((depthFrameDescription.Width * depthFrameDescription.Height)
                    == this.infraredFrameData.Length) &&
                    (depthFrameDescription.Width == this.bitmap.PixelWidth) &&
                    (depthFrameDescription.Height == this.bitmap.PixelHeight))
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyFrameDataToArray(this.depthFrameData);

                    minDepth = depthFrame.DepthMinReliableDistance;
                    maxDepth = depthFrame.DepthMaxReliableDistance;

                    depthFrameProcessed = true;
                }
            }

            // we got a frame, convert and render
            if (depthFrameProcessed)
            {
                ConvertDepthDataToPixels(minDepth, maxDepth);
                RenderPixelArray(this.depthPixels);
            }
        }

        private void ConvertDepthDataToPixels(ushort minDepth, ushort maxDepth)
        {
            int colorPixelIndex = 0;
            // Shape the depth to the range of a byte
            int mapDepthToByte = maxDepth / 256;

            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                // Get the depth for this pixel
                ushort depth = this.depthFrameData[i];

                // To convert to a byte, we're mapping the depth value
                // to the byte range.
                // Values outside the reliable depth range are 
                // mapped to 0 (black).
                byte intensity = (byte)(depth >= minDepth &&
                    depth <= maxDepth ? (depth / mapDepthToByte) : 0);

                this.depthPixels[colorPixelIndex++] = intensity; //Blue
                this.depthPixels[colorPixelIndex++] = intensity; //Green
                this.depthPixels[colorPixelIndex++] = intensity; //Red
                this.depthPixels[colorPixelIndex++] = 255; //Alpha
            }
        }
        private void RenderPixelArray(byte[] pixels)
        {
            pixels.CopyTo(this.bitmap.PixelBuffer);
            this.bitmap.Invalidate();
            FrameDisplayImage.Source = this.bitmap;
        }


        public MainPage()
        {

            CreateFolder();
            
            foreach (var feature in featuresList)
            {
                Features.Add(feature, 0.0f);
            }
            

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            SetupCurrentDisplay(DEFAULT_DISPLAYFRAMETYPE);
            //SetupCurrentDisplay(DisplayFrameType.Depth);

            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Infrared | FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // open the sensor
            this.kinectSensor.Open();

            this.InitializeComponent();

            // Initialize the gesture detection objects for our gestures
            this.gestureDetectorList = new List<GestureDetector>();

            // Create a gesture detector for each body (6 bodies => 6 detectors)
            int maxBodies = this.kinectSensor.BodyFrameSource.BodyCount;
            for (int i = 0; i < maxBodies; ++i)
            {
                GestureResultView result = new GestureResultView("test", i, false, false, 0.0f);
                GestureDetector detector = new GestureDetector(this.kinectSensor, result);
                result.PropertyChanged += GestureResult_PropertyChanged;
                this.gestureDetectorList.Add(detector);
            }


        }

        private void Reader_MultiSourceFrameArrived(MultiSourceFrameReader sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            InfraredFrame infraredFrame = null;
            BodyFrame bodyFrame = null;
            BodyIndexFrame bodyIndexFrame = null;

            IBuffer depthFrameDataBuffer = null;
            IBuffer bodyIndexFrameData = null;
            // Com interface for unsafe byte manipulation
            IBufferByteAccess bodyIndexByteAccess = null;

            using (bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                RegisterGesture(bodyFrame);
            }

            switch (currentDisplayFrameType)
            {
                case DisplayFrameType.Infrared:
                    using (infraredFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
                    {
                        ShowInfraredFrame(infraredFrame);
                    }
                    break;
                case DisplayFrameType.Color:
                    using (colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                    {
                        ShowColorFrame(colorFrame);
                    }
                    break;

                case DisplayFrameType.Depth:
                    using (depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                    {
                        ShowDepthFrame(depthFrame);
                    }
                    break;

                case DisplayFrameType.BodyMask:
                    // Put in a try catch to utilise finally() and clean up frames
                    try
                    {
                        depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                        bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                        colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                        if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null))
                        {
                            return;
                        }

                        // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                        depthFrameDataBuffer = depthFrame.LockImageBuffer();
                        this.coordinateMapper.MapColorFrameToDepthSpaceUsingIBuffer(depthFrameDataBuffer, this.colorMappedToDepthPoints);
                        // Process Color
                        colorFrame.CopyConvertedFrameDataToBuffer(this.bitmap.PixelBuffer, ColorImageFormat.Bgra);
                        // Access the body index frame data directly via LockImageBuffer to avoid making a copy
                        bodyIndexFrameData = bodyIndexFrame.LockImageBuffer();
                        ShowMappedBodyFrame(depthFrame.FrameDescription.Width, depthFrame.FrameDescription.Height, bodyIndexFrameData, bodyIndexByteAccess);

                    }
                    finally
                    {
                        if (depthFrame != null)
                        {
                            depthFrame.Dispose();
                        }
                        if (colorFrame != null)
                        {
                            colorFrame.Dispose();
                        }
                        if (bodyIndexFrame != null)
                        {
                            bodyIndexFrame.Dispose();
                        }

                        if (depthFrameDataBuffer != null)
                        {
                            // We must force a release of the IBuffer in order to ensure that we have dropped all references to it.
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(depthFrameDataBuffer);
                        }
                        if (bodyIndexFrameData != null)
                        {
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(bodyIndexFrameData);
                        }
                        if (bodyIndexByteAccess != null)
                        {
                            System.Runtime.InteropServices.Marshal.ReleaseComObject(bodyIndexByteAccess);
                        }
                    }
                    break;

                case DisplayFrameType.BodyJoints:
                    using (bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                    {
                        ShowBodyJoints(bodyFrame);
                    }
                    break;


                default:
                    break;
            }
        }


        //Creates the folder and the textfile needed for the logging
        public async Task CreateFolder()
        {
            try
            {
                StorageFolder storageFolder = await ApplicationData.Current.LocalFolder.CreateFolderAsync("Test Folder", CreationCollisionOption.OpenIfExists);
                StorageFile testFile = await storageFolder.CreateFileAsync("sample.txt", CreationCollisionOption.ReplaceExisting);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("Error 11: Failed to create folder: " + ex.ToString());
            }
        }


        public async Task log()
        {
            await sem.WaitAsync();
            TimeSpan timestamp = DateTime.Now - CurrentDate;

            try
            {
                StorageFolder storageFolder = await ApplicationData.Current.LocalFolder.GetFolderAsync("Test Folder");
                StorageFile testFile = await storageFolder.GetFileAsync("sample.txt");
                var stream = await testFile.OpenAsync(FileAccessMode.ReadWrite);
                try
                {

                    using (var outputStream = stream.GetOutputStreamAt(stream.Size)) 
                    {
                        using (var dataWriter = new DataWriter(outputStream))
                        {
                            
                            dataWriter.WriteString((order.IndexOf(body_num)+1).ToString()+"-");
                            dataWriter.WriteString(timestamp.ToString() + Environment.NewLine);
                            foreach (var feature in Features)
                            {
                                dataWriter.WriteString(feature.Key + " " + feature.Value.ToString("0.000") + Environment.NewLine);
                            }

                            await dataWriter.StoreAsync();
                            await outputStream.FlushAsync();
                        }
                         
                    }
                    stream.Dispose(); 
                   
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine("Error 11 - Writing on file " + ex.ToString());
                }
                
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("Error 10 - Writing on file " + ex.ToString());
            }
            finally
            {
                sem.Release();
            }
        }



        //what to do once a gesture's confidence is modified
        private void GestureResult_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            GestureResultView result = sender as GestureResultView;
            Features[result.GestureName] = result.Confidence;
        }

        //Checks if the Kinect is connected
        private void Sensor_IsAvailableChanged(KinectSensor sender, IsAvailableChangedEventArgs args)
        {
            this.StatusText = this.kinectSensor.IsAvailable ?
                 "Running" : "Not Available";
        }


        private void SetupCurrentDisplay(DisplayFrameType newDisplayFrameType)
        {
            currentDisplayFrameType = newDisplayFrameType;
            // Frames used by more than one type are declared outside the switch
            FrameDescription colorFrameDescription = null;

            // reset the display methods
            if (this.BodyJointsGrid != null)
            {
                this.BodyJointsGrid.Visibility = Visibility.Collapsed;
            }
            if (this.FrameDisplayImage != null)
            {
                this.FrameDisplayImage.Source = null;
            }

            switch (currentDisplayFrameType)
            {
                //if the Infrared button is clicked
                case DisplayFrameType.Infrared:
                    FrameDescription infraredFrameDescription = this.kinectSensor.InfraredFrameSource.FrameDescription;
                    this.CurrentFrameDescription = infraredFrameDescription;
                    // allocate space to put the pixels being 
                    // received and converted
                    this.infraredFrameData = new ushort[infraredFrameDescription.Width * infraredFrameDescription.Height];
                    this.infraredPixels = new byte[infraredFrameDescription.Width * infraredFrameDescription.Height * BytesPerPixel];
                    this.bitmap = new WriteableBitmap(infraredFrameDescription.Width,infraredFrameDescription.Height);
                    break;

                //if the Color button is clicked
                case DisplayFrameType.Color:
                    colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                    this.CurrentFrameDescription = colorFrameDescription;
                    // create the bitmap to display
                    this.bitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height);
                    break;

                //if the Depth button is clicked
                case DisplayFrameType.Depth:
                    FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
                    this.CurrentFrameDescription = depthFrameDescription;
                    // allocate space to put the pixels being 
                    // received and converted
                    this.depthFrameData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height];
                    this.depthPixels = new byte[depthFrameDescription.Width * depthFrameDescription.Height * BytesPerPixel];
                    this.bitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height);
                    break;

                //if the Body Mask button is clicked
                case DisplayFrameType.BodyMask:
                    colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                    this.CurrentFrameDescription = colorFrameDescription;
                    // allocate space to put the pixels being received and converted
                    this.colorMappedToDepthPoints = new DepthSpacePoint[colorFrameDescription.Width * colorFrameDescription.Height];
                    this.bitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height);
                    break;


                //if the Joint button is clicked
                case DisplayFrameType.BodyJoints:
                    // instantiate a new Canvas
                    this.drawingCanvas = new Canvas();
                    // set the clip rectangle to prevent rendering outside the canvas
                    this.drawingCanvas.Clip = new RectangleGeometry();
                    this.drawingCanvas.Clip.Rect = new Rect(0.0, 0.0, this.BodyJointsGrid.Width,this.BodyJointsGrid.Height);
                    this.drawingCanvas.Width = this.BodyJointsGrid.Width;
                    this.drawingCanvas.Height = this.BodyJointsGrid.Height;
                    // reset the body joints grid
                    this.BodyJointsGrid.Visibility = Visibility.Visible;
                    this.BodyJointsGrid.Children.Clear();
                    // add canvas to DisplayGrid
                    this.BodyJointsGrid.Children.Add(this.drawingCanvas);
                    bodiesManager = new BodiesManager(this.coordinateMapper, this.drawingCanvas, this.kinectSensor.BodyFrameSource.BodyCount);
                    break;

                default:
                    break;
            }
        }

        //Choosing the Infrared view option
        private void InfraredButton_Click(object sender, RoutedEventArgs e)
        {
            SetupCurrentDisplay(DisplayFrameType.Infrared);
        }

        //Choosing the Color view option
        private void ColorButton_Click(object sender, RoutedEventArgs e)
        {
            SetupCurrentDisplay(DisplayFrameType.Color);
        }
        
        //Choosing the Depth view option
        private void DepthButton_Click(object sender, RoutedEventArgs e)
        {
            SetupCurrentDisplay(DisplayFrameType.Depth);
        }

        //Choosing the Body masking option (isolating the body)
        private void BodyMask_Click(object sender, RoutedEventArgs e)
        {
            SetupCurrentDisplay(DisplayFrameType.BodyMask);
        }

        //Choosing the Skeletal view option
        private void BodyJointsButton_Click(object sender, RoutedEventArgs e)
        {
            SetupCurrentDisplay(DisplayFrameType.BodyJoints);
        }


        //Register the detected gesture + logs each gesture
        private async void RegisterGesture(BodyFrame bodyFrame)
        {
            bool dataReceived = false;
            Body[] bodies = null;

            if (bodyFrame != null)
            {
                if (bodies == null)
                {
                    // Creates an array of 6 bodies, which is the max number of bodies the Kinect can track simultaneously
                    bodies = new Body[bodyFrame.BodyCount];
                }

                // The first time GetAndRefreshBodyData is called, allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array, those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(bodies);
                dataReceived = true;
            }

            if (dataReceived)
            {
                
                for(int i = 0; i < 6; i++)
                {
                    if (bodies[i].IsTracked)
                    {
                        tracked[i]=bodies[i].TrackingId;
                        if (!order.Contains(i))
                        {
                            order.Add(i);
                        }
                    }
                }

                // We may have lost/acquired bodies, so update the corresponding gesture detectors
                if (bodies != null)
                {
                    // Loop through all bodies to see if any of the gesture detectors need to be updated
                    for (int i = 0; i < bodyFrame.BodyCount; ++i)
                    {
                        Body body = bodies[i];
                        ulong trackingId = body.TrackingId;

                        // If the current body TrackingId changed, update the corresponding gesture detector with the new value
                        if (trackingId != this.gestureDetectorList[i].TrackingId)
                        {
                            this.gestureDetectorList[i].TrackingId = trackingId;

                            // If the current body is tracked, unpause its detector to get VisualGestureBuilderFrameArrived events
                            // If the current body is NOT tracked, pause its detector so we don't waste resources trying to get invalid gesture results
                            this.gestureDetectorList[i].IsPaused = trackingId == 0;
                        }


                        
                    }

                    for (int i = 0; i < 6; i++)
                    {
                        if (this.gestureDetectorList[i].TrackingId == tracked[i])
                        {
                            body_num = i;
                            await log();
                        }
                    }


                }
                this.GestureVisual1.Text = (DateTime.Now - CurrentDate).ToString();
            }
        }

        [Guid("905a0fef-bc53-11df-8c49-001e4fc686da"),
                 InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        interface IBufferByteAccess
        {
            unsafe void Buffer(out byte* pByte);
        }
    }
}
