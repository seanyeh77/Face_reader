//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

using System;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using Windows.ApplicationModel;
using Windows.Devices.Enumeration;
using Windows.Devices.Sensors;
using Windows.Foundation;
using Windows.Foundation.Metadata;
using Windows.Graphics.Display;
using Windows.Graphics.Imaging;
using Windows.Media;
using Windows.Media.Core;
using Windows.Media.Capture;
using Windows.Media.MediaProperties;
using Windows.Phone.UI.Input;
using Windows.Storage;
using Windows.Storage.FileProperties;
using Windows.Storage.Streams;
using Windows.System.Display;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Shapes;
using Windows.Media.FaceAnalysis;
using Windows.UI;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Collections;
using System.IO;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Http.Internal;
using Windows.System;
using RabbitMQ.Client.Events;
using System.Threading.Channels;
using RabbitMQ.Client;
using System.Text.RegularExpressions;
using System.Security.Cryptography;
using System.Text;

namespace Face_reader
{

    public sealed partial class MainPage : Page
    {
        // Receive notifications about rotation of the device and UI and apply any necessary rotation to the preview stream and UI controls
        private readonly DisplayInformation _displayInformation = DisplayInformation.GetForCurrentView();
        private readonly SimpleOrientationSensor _orientationSensor = SimpleOrientationSensor.GetDefault();
        private DisplayOrientations _displayOrientation = DisplayOrientations.Portrait;
        // Rotation metadata to apply to the preview stream and recorded videos (MF_MT_VIDEO_ROTATION)
        // Reference: http://msdn.microsoft.com/en-us/library/windows/apps/xaml/hh868174.aspx
        private static readonly Guid RotationKey = new Guid("C380465D-2271-428C-9B83-ECEA3B4A85C1");
        //public static HttpClient client = new HttpClient() { BaseAddress = new Uri("https://localhost:7021/") };
        public static HttpClient client1 = new HttpClient() { BaseAddress = new Uri("http://192.168.50.237/") };
        public static HttpClient client = new HttpClient() { BaseAddress = new Uri("https://userdatawebapi20220829195800.azurewebsites.net/") };

        // Prevent the screen from sleeping while the camera is running
        private readonly DisplayRequest _displayRequest = new DisplayRequest();

        // For listening to media property changes
        private readonly SystemMediaTransportControls _systemMediaControls = SystemMediaTransportControls.GetForCurrentView();

        // MediaCapture and its state variables
        private MediaCapture _mediaCapture1;
        private MediaCapture _mediaCapture2;
        private IMediaEncodingProperties _previewProperties1;
        private IMediaEncodingProperties _previewProperties2;
        private bool _isInitialized;
        private bool _isRecording;

        private FaceDetectionEffect _faceDetectionEffect1;
        private FaceDetectionEffect _faceDetectionEffect2;
        private DateTime dateTime1;
        private bool temp1 = true;
        private bool temp2 = false;
        ConnectionFactory factory = new ConnectionFactory() { HostName = "localhost", UserName = "guest", Password = "guest" };
        private EventingBasicConsumer consumer;
        private IModel channel;
        private IConnection connection;
        #region Constructor, lifecycle and navigation

        public MainPage()
        {
            try
            {
                InitializeComponent();
                // Useful to know when to initialize/clean up the camera
                Application.Current.Suspending += Application_Suspending;//關閉事件
                Application.Current.Resuming += Application_Resuming;
            }
            catch(Exception ex)
            {
                left.Text = "發生錯誤";
                right.Text = "發生錯誤";
            }
        }

        private async void Application_Suspending(object sender, SuspendingEventArgs e)
        {
            // Handle global application events only if this page is active
            if (Frame.CurrentSourcePageType == typeof(MainPage))
            {
                var deferral = e.SuspendingOperation.GetDeferral();

                await CleanupCameraAsync();

                deferral.Complete();
            }
        }

        private async void Application_Resuming(object sender, object o)
        {
            // Handle global application events only if this page is active
            if (Frame.CurrentSourcePageType == typeof(MainPage))
            {
                await InitializeCameraAsync();
            }
        }

        protected override async void OnNavigatedTo(NavigationEventArgs e)
        {
            try
            {
                connection = factory.CreateConnection();
                channel = connection.CreateModel();
                channel.QueueDeclare(queue: "hello",
                                    durable: false,
                                    exclusive: false,
                                    autoDelete: false,
                                    arguments: null);

                consumer = new EventingBasicConsumer(channel);
                consumer.Received += OnQueueReceived;
                channel.BasicConsume(queue: "hello",
                                     autoAck: true,
                                     consumer: consumer);
                SetupUiAsync();

                await InitializeCameraAsync();
            }
            catch(Exception ex)
            {
                await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { left.Text = "辨識錯誤"; });
                await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { right.Text = "辨識錯誤"; });
            }

        }
        private void OnQueueReceived(Object model, BasicDeliverEventArgs ea)
        {
                Task.Run(async () => {
                    if (_previewProperties1 != null && temp2)
                    {
                        HttpContent contentPost = new StringContent("", Encoding.UTF8, "application/json");
                        try
                        {
                            HttpResponseMessage response1 = await client1.GetAsync("open");
                        }
                        catch
                        {
                            right.Text = "";
                        }
                        var lowLagCapture = await _mediaCapture2.PrepareLowLagPhotoCaptureAsync(ImageEncodingProperties.CreateUncompressed(MediaPixelFormat.Bgra8));
                        CapturedPhoto capturedPhoto = await lowLagCapture.CaptureAsync();
                        SoftwareBitmap softwareBitmap = capturedPhoto.Frame.SoftwareBitmap;
                        byte[] imageBytes = await EncodedBytes(softwareBitmap, BitmapEncoder.JpegEncoderId);
                        MemoryStream stream = new MemoryStream(imageBytes);
                        IFormFile file = new FormFile(stream, 0, imageBytes.Length, "image.jpeg", "image.jpeg");
                        var formdata = new MultipartFormDataContent();
                        formdata.Add(new StreamContent(file.OpenReadStream()), "userdata", file.FileName);
                        formdata.Add(new StringContent("out"), "state");
                        try
                        {
                            HttpResponseMessage response = await client.PostAsync("UserLog/face", formdata);
                            if (response.IsSuccessStatusCode)
                            {
                                string users = await response.Content.ReadAsStringAsync();
                                if (users != "null")
                                {
                                    users = users.Replace("[", "");
                                    users = users.Replace("]", "");
                                    users = users.Replace("\"", "");
                                    users = users.Replace(",", "\n");
                                    await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { right.Text = $"{users}\n簽退"; });
                                }
                                else
                                {
                                    await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { right.Text = "辨識錯誤"; });
                                }
                            }
                        }
                        catch (Exception ex)
                        {
                            await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { right.Text = ""; });
                        }
                        await lowLagCapture.FinishAsync();
                    }
                    else
                    {
                        await this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () => { right.Text = ""; });
                        HttpContent contentPost = new StringContent("", Encoding.UTF8, "application/json");
                        try
                        {
                            HttpResponseMessage response1 = await client1.GetAsync("close");
                        }
                        catch
                        {
                            right.Text = "";
                        }
                    }
                    
                });
            
        }
        #endregion Constructor, lifecycle and navigation


        #region Event handlers

        /// <summary>
        /// In the event of the app being minimized this method handles media property change events. If the app receives a mute
        /// notification, it is no longer in the foregroud.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        private async void SystemMediaControls_PropertyChanged(SystemMediaTransportControls sender, SystemMediaTransportControlsPropertyChangedEventArgs args)
        {
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
            {
                // Only handle this event if this page is currently being displayed
                if (args.Property == SystemMediaTransportControlsProperty.SoundLevel && Frame.CurrentSourcePageType == typeof(MainPage))
                {
                    // Check to see if the app is being muted. If so, it is being minimized.
                    // Otherwise if it is not initialized, it is being brought into focus.
                    if (sender.SoundLevel == SoundLevel.Muted)
                    {
                        await CleanupCameraAsync();
                    }
                    else if (!_isInitialized)
                    {
                        await InitializeCameraAsync();
                    }
                }
            });
        }
        private async void FaceDetectionEffect_FaceDetected1(FaceDetectionEffect sender, FaceDetectedEventArgs args)
        {
            // Ask the UI thread to render the face bounding boxes
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () => HighlightDetectedFaces1(args.ResultFrame.DetectedFaces));
        }
        private async void FaceDetectionEffect_FaceDetected2(FaceDetectionEffect sender, FaceDetectedEventArgs args)
        {
            // Ask the UI thread to render the face bounding boxes
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () => HighlightDetectedFaces2(args.ResultFrame.DetectedFaces));
        }
        #endregion Event handlers


        #region MediaCapture methods

        /// <summary>
        /// Initializes the MediaCapture, registers events, gets camera device information for mirroring and rotating, starts preview and unlocks the UI
        /// </summary>
        /// <returns></returns>
        private async Task InitializeCameraAsync()
        {
            Debug.WriteLine("InitializeCameraAsync");
            dateTime1 = DateTime.Now;
            temp1 = true;
            if (_mediaCapture1 == null)
            {
                // Attempt to get the front camera if one is available, but use any camera device if not
                var cameraDevice1 = await FindFirstCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel.Front);
                var cameraDevice2 = await FindSecondCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel.Front);

                if (cameraDevice1 == null)
                {
                    Debug.WriteLine("No camera1 device found!");
                    return;
                }
                if (cameraDevice2 == null)
                {
                    Debug.WriteLine("No camera2 device found!");
                    return;
                }
                // Create MediaCapture and its settings
                _mediaCapture1 = new MediaCapture();
                var settings1 = new MediaCaptureInitializationSettings { VideoDeviceId = cameraDevice1.Id };
                _mediaCapture2 = new MediaCapture();
                var settings2 = new MediaCaptureInitializationSettings { VideoDeviceId = cameraDevice2.Id };

                // Initialize MediaCapture
                try
                {
                    await _mediaCapture1.InitializeAsync(settings1);
                    await _mediaCapture2.InitializeAsync(settings2);
                    _isInitialized = true;
                }
                catch (UnauthorizedAccessException)
                {
                    Debug.WriteLine("The app was denied access to the camera");
                }

                // If initialization succeeded, start the preview
                if (_isInitialized)
                {
                    await StartPreviewAsync();
                    await CreateFaceDetectionEffectAsync1();
                    await CreateFaceDetectionEffectAsync2();
                }
            }
        }

        /// <summary>
        /// Starts the preview and adjusts it for for rotation and mirroring after making a request to keep the screen on
        /// </summary>
        /// <returns></returns>
        private async Task StartPreviewAsync()
        {
            // Prevent the device from sleeping while the preview is running
            _displayRequest.RequestActive();

            // Set the preview source in the UI and mirror it if necessary
            PreviewControl1.Source = _mediaCapture1;
            PreviewControl2.Source = _mediaCapture2;

            // Start the preview
            await _mediaCapture1.StartPreviewAsync();
            await _mediaCapture2.StartPreviewAsync();
            _previewProperties1 = _mediaCapture1.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview);
            _previewProperties2 = _mediaCapture2.VideoDeviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview);

            // Initialize the preview to the current orientation
            if (_previewProperties1 != null)
            {
                _displayOrientation = _displayInformation.CurrentOrientation;
            }
        }

        /// <summary>
        /// Stops the preview and deactivates a display request, to allow the screen to go into power saving modes
        /// </summary>
        /// <returns></returns>
        private async Task StopPreviewAsync()
        {
            // Stop the preview
            _previewProperties1 = null;
            await _mediaCapture1.StopPreviewAsync();
            await _mediaCapture2.StopPreviewAsync();

            // Use the dispatcher because this method is sometimes called from non-UI threads
            await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                // Cleanup the UI
                PreviewControl1.Source = null;
                PreviewControl2.Source = null;

                // Allow the device screen to sleep now that the preview is stopped
                _displayRequest.RequestRelease();
            });
        }

        /// <summary>
        /// Adds face detection to the preview stream, registers for its events, enables it, and gets the FaceDetectionEffect instance
        /// </summary>
        /// <returns></returns>
        private async Task CreateFaceDetectionEffectAsync1()
        {
            // Create the definition, which will contain some initialization settings
            var definition = new FaceDetectionEffectDefinition();

            // To ensure preview smoothness, do not delay incoming samples
            definition.SynchronousDetectionEnabled = false;

            // In this scenario, choose detection speed over accuracy
            definition.DetectionMode = FaceDetectionMode.HighPerformance;

            // Add the effect to the preview stream
            _faceDetectionEffect1 = (FaceDetectionEffect)await _mediaCapture1.AddVideoEffectAsync(definition, MediaStreamType.VideoPreview);

            // Register for face detection events
            _faceDetectionEffect1.FaceDetected += FaceDetectionEffect_FaceDetected1;

            // Choose the shortest interval between detection events
            _faceDetectionEffect1.DesiredDetectionInterval = TimeSpan.FromMilliseconds(0);

            // Start detecting faces
            _faceDetectionEffect1.Enabled = true;
        }
        private async Task CreateFaceDetectionEffectAsync2()
        {
            // Create the definition, which will contain some initialization settings
            var definition = new FaceDetectionEffectDefinition();

            // To ensure preview smoothness, do not delay incoming samples
            definition.SynchronousDetectionEnabled = false;

            // In this scenario, choose detection speed over accuracy
            definition.DetectionMode = FaceDetectionMode.HighPerformance;

            // Add the effect to the preview stream
            _faceDetectionEffect2 = (FaceDetectionEffect)await _mediaCapture2.AddVideoEffectAsync(definition, MediaStreamType.VideoPreview);

            // Register for face detection events
            _faceDetectionEffect2.FaceDetected += FaceDetectionEffect_FaceDetected2;

            // Choose the shortest interval between detection events
            _faceDetectionEffect2.DesiredDetectionInterval = TimeSpan.FromMilliseconds(33);

            // Start detecting faces
            _faceDetectionEffect2.Enabled = true;
        }
        /// <summary>
        ///  Disables and removes the face detection effect, and unregisters the event handler for face detection
        /// </summary>
        /// <returns></returns>
        private async Task CleanUpFaceDetectionEffectAsync1()
        {
            // Disable detection
            _faceDetectionEffect1.Enabled = false;

            // Unregister the event handler
            _faceDetectionEffect1.FaceDetected -= FaceDetectionEffect_FaceDetected1;

            // Remove the effect (see ClearEffectsAsync method to remove all effects from a stream)
            await _mediaCapture1.RemoveEffectAsync(_faceDetectionEffect1);

            // Clear the member variable that held the effect instance
            _faceDetectionEffect1 = null;
        }
        private async Task CleanUpFaceDetectionEffectAsync2()
        {
            // Disable detection
            _faceDetectionEffect2.Enabled = false;

            // Unregister the event handler
            _faceDetectionEffect2.FaceDetected -= FaceDetectionEffect_FaceDetected2;

            // Remove the effect (see ClearEffectsAsync method to remove all effects from a stream)
            await _mediaCapture2.RemoveEffectAsync(_faceDetectionEffect2);

            // Clear the member variable that held the effect instance
            _faceDetectionEffect2 = null;
        }
        /// <summary>
        /// Stops recording a video
        /// </summary>
        /// <returns></returns>
        private async Task StopRecordingAsync()
        {
            Debug.WriteLine("Stopping recording...");

            _isRecording = false;
            await _mediaCapture1.StopRecordAsync();
            await _mediaCapture2.StopRecordAsync();

            Debug.WriteLine("Stopped recording!");
        }

        /// <summary>
        /// Cleans up the camera resources (after stopping any video recording and/or preview if necessary) and unregisters from MediaCapture events
        /// </summary>
        /// <returns></returns>
        private async Task CleanupCameraAsync()
        {
            Debug.WriteLine("CleanupCameraAsync");

            if (_isInitialized)
            {
                // If a recording is in progress during cleanup, stop it to save the recording
                if (_isRecording)
                {
                    await StopRecordingAsync();
                }

                if (_faceDetectionEffect1 != null)
                {
                    await CleanUpFaceDetectionEffectAsync1();
                }

                if (_previewProperties1 != null)
                {
                    // The call to stop the preview is included here for completeness, but can be
                    // safely removed if a call to MediaCapture.Dispose() is being made later,
                    // as the preview will be automatically stopped at that point
                    await StopPreviewAsync();
                }

                _isInitialized = false;
            }

            if (_mediaCapture1 != null)
            {
                //_mediaCapture.RecordLimitationExceeded -= MediaCapture_RecordLimitationExceeded;
                //_mediaCapture.Failed -= MediaCapture_Failed;
                _mediaCapture1.Dispose();
                _mediaCapture2.Dispose();
                _mediaCapture1 = null;
                _mediaCapture2 = null;
            }
        }

        #endregion MediaCapture methods


        #region Helper functions

        /// <summary>
        /// Attempts to lock the page orientation, hide the StatusBar (on Phone) and registers event handlers for hardware buttons and orientation sensors
        /// </summary>
        /// <returns></returns>
        private void SetupUiAsync()
        {
            // Attempt to lock page to landscape orientation to prevent the CaptureElement from rotating, as this gives a better experience
            DisplayInformation.AutoRotationPreferences = DisplayOrientations.Landscape;
            RegisterEventHandlers();
        }
        /// <summary>
        /// Registers event handlers for hardware buttons and orientation sensors, and performs an initial update of the UI rotation
        /// </summary>
        private void RegisterEventHandlers()
        {
            _systemMediaControls.PropertyChanged += SystemMediaControls_PropertyChanged;
        }

        /// <summary>
        /// Unregisters event handlers for hardware buttons and orientation sensors
        /// </summary>
        private void UnregisterEventHandlers()
        {
            _systemMediaControls.PropertyChanged -= SystemMediaControls_PropertyChanged;
        }

        /// <summary>
        /// Attempts to find and return a device mounted on the panel specified, and on failure to find one it will return the first device listed
        /// </summary>
        /// <param name="desiredPanel">The desired panel on which the returned device should be mounted, if available</param>
        /// <returns></returns>
        private static async Task<DeviceInformation> FindFirstCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel desiredPanel)
        {
            // Get available devices for capturing pictures
            var allVideoDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);

            // Get the desired camera by panel
            DeviceInformation desiredDevice = allVideoDevices.FirstOrDefault(x => x.EnclosureLocation != null && x.EnclosureLocation.Panel == desiredPanel);

            // If there is no device mounted on the desired panel, return the first device found
            return desiredDevice ?? allVideoDevices.FirstOrDefault();
        }
        private static async Task<DeviceInformation> FindSecondCameraDeviceByPanelAsync(Windows.Devices.Enumeration.Panel desiredPanel)
        {
            // Get available devices for capturing pictures
            var allVideoDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);
            // If there is no device mounted on the desired panel, return the first device found
            return allVideoDevices[1];
        }


        #endregion Helper functions


        #region Rotation helpers

        /// <summary>
        /// Uses the current display orientation to calculate the rotation transformation to apply to the face detection bounding box canvas
        /// and mirrors it if the preview is being mirrored
        /// </summary>
        private void SetFacesCanvas1Rotation1()
        {
            // Apply the rotation
            var transform1 = new RotateTransform();
            FacesCanvas1.RenderTransform = transform1;

            var previewArea1 = GetPreviewStreamRectInControl(_previewProperties1 as VideoEncodingProperties, PreviewControl1);
            FacesCanvas1.Width = previewArea1.Width;
            FacesCanvas1.Height = previewArea1.Height;

            Canvas.SetLeft(FacesCanvas1, previewArea1.X);
            Canvas.SetTop(FacesCanvas1, previewArea1.Y);
        }
        private void SetFacesCanvas1Rotation2()
        {
            // Apply the rotation
            var transform2 = new RotateTransform();
            FacesCanvas2.RenderTransform = transform2;

            var previewArea2 = GetPreviewStreamRectInControl(_previewProperties2 as VideoEncodingProperties, PreviewControl2);
            FacesCanvas2.Width = previewArea2.Width;
            FacesCanvas2.Height = previewArea2.Height;

            Canvas.SetLeft(FacesCanvas2, previewArea2.X);
            Canvas.SetTop(FacesCanvas2, previewArea2.Y);
        }
        #endregion Rotation helpers


        #region Face detection helpers

        /// <summary>
        /// Iterates over all detected faces, creating and adding Rectangles to the FacesCanvas1 as face bounding boxes
        /// </summary>
        /// <param name="faces">The list of detected faces from the FaceDetected event of the effect</param>
        private async Task HighlightDetectedFaces1(IReadOnlyList<DetectedFace> faces)
        {
            // Remove any existing rectangles from previous events
            FacesCanvas1.Children.Clear();
            
            // For each detected face
            for (int i = 0; i < faces.Count; i++)
            {
                // Face coordinate units are preview resolution pixels, which can be a different scale from our display resolution, so a conversion may be necessary
                Rectangle faceBoundingBox = ConvertPreviewToUiRectangle1(faces[i].FaceBox);
                // Set bounding box stroke properties
                faceBoundingBox.StrokeThickness = 5;

                // Highlight the first face in the set
                faceBoundingBox.Stroke = (i == 0 ? new SolidColorBrush(Colors.Blue) : new SolidColorBrush(Colors.DeepSkyBlue));

                // Add grid to canvas containing all face UI objects
                FacesCanvas1.Children.Add(faceBoundingBox);
            }

            if (!faces.Any())
            {
                dateTime1 = DateTime.Now;
                left.Text = "waiting";
            }
            TimeSpan time1 = DateTime.Now - dateTime1;
            timeleft.Text = (3 - time1.Seconds).ToString();
            if (!faces.Any() || time1 <= TimeSpan.FromSeconds(0))
            {

                timeleft.Visibility = Visibility.Collapsed;
            }
            else
            {
                timeleft.Visibility = Visibility.Visible;
            }

            //送出POST
            if (DateTime.Now - dateTime1 > TimeSpan.FromSeconds(1)&&temp1)
            {
                temp1 = false;
                var lowLagCapture = await _mediaCapture1.PrepareLowLagPhotoCaptureAsync(ImageEncodingProperties.CreateUncompressed(MediaPixelFormat.Bgra8));
                CapturedPhoto capturedPhoto = await lowLagCapture.CaptureAsync();
                SoftwareBitmap softwareBitmap = capturedPhoto.Frame.SoftwareBitmap;
                byte[] imageBytes = await EncodedBytes(softwareBitmap, BitmapEncoder.JpegEncoderId);
                MemoryStream stream = new MemoryStream(imageBytes);
                IFormFile file = new FormFile(stream, 0, imageBytes.Length, "image.jpeg", "image.jpeg");
                var formdata = new MultipartFormDataContent();
                formdata.Add(new StreamContent(file.OpenReadStream()), "userdata", file.FileName);
                formdata.Add(new StringContent("in"), "state");
                try
                {
                    HttpResponseMessage response = await client.PostAsync("UserLog/face", formdata);
                    if (response.IsSuccessStatusCode)
                    {
                        string users = await response.Content.ReadAsStringAsync();
                        if (users != "null")
                        {
                            users = users.Replace("[", "");
                            users = users.Replace("]", "");
                            users = users.Replace("\"", "");
                            users = users.Replace(",", "\n");
                            left.Text = $"{users}\n簽到";
                            HttpContent contentPost = new StringContent("", Encoding.UTF8, "application/json");
                            try
                            {
                                HttpResponseMessage response1 = await client1.GetAsync("open");
                            }
                            catch
                            {
                                right.Text = "";
                            }
                        }
                        else
                        {
                            left.Text = "辨識錯誤";
                        }
                    }

                }
                catch (Exception ex)
                {
                    left.Text = "";
                }
                await lowLagCapture.FinishAsync();
                dateTime1 = DateTime.Now;
                temp1 = true;
            }
            // Update the face detection bounding box canvas orientation
            SetFacesCanvas1Rotation1();
        }
        private async Task HighlightDetectedFaces2(IReadOnlyList<DetectedFace> faces)
        {
            // Remove any existing rectangles from previous events
            FacesCanvas2.Children.Clear();
            // For each detected face
            if (faces.Any())
            {
                temp2 = true;
            }
            else
            {
                temp2 = false;
            }
            for (int i = 0; i < faces.Count; i++)
            {
                // Face coordinate units are preview resolution pixels, which can be a different scale from our display resolution, so a conversion may be necessary
                Rectangle faceBoundingBox = ConvertPreviewToUiRectangle2(faces[i].FaceBox);
                faceBoundingBox.Height = faceBoundingBox.Height;
                faceBoundingBox.Width = faceBoundingBox.Width;
                // Set bounding box stroke properties
                faceBoundingBox.StrokeThickness = 5;

                // Highlight the first face in the set
                faceBoundingBox.Stroke = (i == 0 ? new SolidColorBrush(Colors.Blue) : new SolidColorBrush(Colors.DeepSkyBlue));

                // Add grid to canvas containing all face UI objects
                FacesCanvas2.Children.Add(faceBoundingBox);
            }
            // Update the face detection bounding box canvas orientation
            SetFacesCanvas1Rotation2();
        }
        /// <summary>
        /// Takes face information defined in preview coordinates and returns one in UI coordinates, taking
        /// into account the position and size of the preview control.
        /// </summary>
        /// <param name="faceBoxInPreviewCoordinates">Face coordinates as retried from the FaceBox property of a DetectedFace, in preview coordinates.</param>
        /// <returns>Rectangle in UI (CaptureElement) coordinates, to be used in a Canvas control.</returns>
        private Rectangle ConvertPreviewToUiRectangle1(BitmapBounds faceBoxInPreviewCoordinates)
        {
            var result = new Rectangle();
            var previewStream = _previewProperties1 as VideoEncodingProperties;
            // If there is no available information about the preview, return an empty rectangle, as re-scaling to the screen coordinates will be impossible
            if (previewStream == null)
            {
                return result;
            }
            // Similarly, if any of the dimensions is zero (which would only happen in an error case) return an empty rectangle
            if (previewStream.Width == 0 || previewStream.Height == 0)
            {
                return result;
            }

            double streamWidth = previewStream.Width;
            double streamHeight = previewStream.Height;

            // For portrait orientations, the width and height need to be swapped
            if (_displayOrientation == DisplayOrientations.Portrait || _displayOrientation == DisplayOrientations.PortraitFlipped)
            {
                streamHeight = previewStream.Width;
                streamWidth = previewStream.Height;
            }

            // Get the rectangle that is occupied by the actual video feed
            var previewInUI = GetPreviewStreamRectInControl(previewStream, PreviewControl1);

            // Scale the width and height from preview stream coordinates to window coordinates
            result.Width = (faceBoxInPreviewCoordinates.Width / streamWidth) * previewInUI.Width;
            result.Height = (faceBoxInPreviewCoordinates.Height / streamHeight) * previewInUI.Height;

            // Scale the X and Y coordinates from preview stream coordinates to window coordinates
            var x = (faceBoxInPreviewCoordinates.X / streamWidth) * previewInUI.Width;
            var y = (faceBoxInPreviewCoordinates.Y / streamHeight) * previewInUI.Height;

            Canvas.SetLeft(result, x);
            Canvas.SetTop(result, y);

            return result;
        }
        private Rectangle ConvertPreviewToUiRectangle2(BitmapBounds faceBoxInPreviewCoordinates)//不可刪
        {
            var result = new Rectangle();
            var previewStream = _previewProperties2 as VideoEncodingProperties;

            // If there is no available information about the preview, return an empty rectangle, as re-scaling to the screen coordinates will be impossible
            if (previewStream == null)
            {
                return result;
            }
            // Similarly, if any of the dimensions is zero (which would only happen in an error case) return an empty rectangle
            if (previewStream.Width == 0 || previewStream.Height == 0)
            {
                return result;
            }

            double streamWidth = previewStream.Width;
            double streamHeight = previewStream.Height;

            // For portrait orientations, the width and height need to be swapped
            if (_displayOrientation == DisplayOrientations.Portrait || _displayOrientation == DisplayOrientations.PortraitFlipped)
            {
                streamHeight = previewStream.Width;
                streamWidth = previewStream.Height;
            }

            // Get the rectangle that is occupied by the actual video feed
            var previewInUI = GetPreviewStreamRectInControl(previewStream, PreviewControl1);
            // Scale the width and height from preview stream coordinates to window coordinates
            result.Width = (faceBoxInPreviewCoordinates.Width / streamWidth) * previewInUI.Width;
            result.Height = (faceBoxInPreviewCoordinates.Height / streamHeight) * previewInUI.Height;

            // Scale the X and Y coordinates from preview stream coordinates to window coordinates
            var x = (faceBoxInPreviewCoordinates.X / streamWidth) * previewInUI.Width;
            var y = (faceBoxInPreviewCoordinates.Y / streamHeight) * previewInUI.Height;
            Canvas.SetLeft(result, x);
            Canvas.SetTop(result, y);

            return result;
        }

        /// <summary>
        /// Calculates the size and location of the rectangle that contains the preview stream within the preview control, when the scaling mode is Uniform
        /// </summary>
        /// <param name="previewResolution">The resolution at which the preview is running</param>
        /// <param name="previewControl">The control that is displaying the preview using Uniform as the scaling mode</param>
        /// <returns></returns>
        public Rect GetPreviewStreamRectInControl(VideoEncodingProperties previewResolution, CaptureElement previewControl)//不可刪
        {
            var result = new Rect();

            // In case this function is called before everything is initialized correctly, return an empty result
            if (previewControl == null || previewControl.ActualHeight < 1 || previewControl.ActualWidth < 1 ||
                previewResolution == null || previewResolution.Height == 0 || previewResolution.Width == 0)
            {
                return result;
            }

            var streamWidth = previewResolution.Width;
            var streamHeight = previewResolution.Height;

            // For portrait orientations, the width and height need to be swapped
            if (_displayOrientation == DisplayOrientations.Portrait || _displayOrientation == DisplayOrientations.PortraitFlipped)
            {
                streamWidth = previewResolution.Height;
                streamHeight = previewResolution.Width;
            }

            // Start by assuming the preview display area in the control spans the entire width and height both (this is corrected in the next if for the necessary dimension)
            result.Width = previewControl.ActualWidth;
            result.Height = previewControl.ActualHeight;

            // If UI is "wider" than preview, letterboxing will be on the sides
            if ((previewControl.ActualWidth / previewControl.ActualHeight > streamWidth / (double)streamHeight))
            {
                var scale = previewControl.ActualHeight / streamHeight;
                var scaledWidth = streamWidth * scale;

                result.X = (previewControl.ActualWidth - scaledWidth) / 2.0;
                result.Width = scaledWidth;
            }
            else // Preview stream is "wider" than UI, so letterboxing will be on the top+bottom
            {
                var scale = previewControl.ActualWidth / streamWidth;
                var scaledHeight = streamHeight * scale;

                result.Y = (previewControl.ActualHeight - scaledHeight) / 2.0;
                result.Height = scaledHeight;
            }

            return result;
        }

        #endregion
        private async Task<byte[]> EncodedBytes(SoftwareBitmap soft, Guid encoderId)
        {
            byte[] array = null;

            // First: Use an encoder to copy from SoftwareBitmap to an in-mem stream (FlushAsync)
            // Next:  Use ReadAsync on the in-mem stream to get byte[] array

            using (var ms = new InMemoryRandomAccessStream())
            {
                BitmapEncoder encoder = await BitmapEncoder.CreateAsync(encoderId, ms);
                encoder.SetSoftwareBitmap(soft);

                try
                {
                    await encoder.FlushAsync();
                }
                catch (Exception ex) { return new byte[0]; }

                array = new byte[ms.Size];
                await ms.ReadAsync(array.AsBuffer(), (uint)ms.Size, InputStreamOptions.None);
            }
            return array;
        }
    }
}
