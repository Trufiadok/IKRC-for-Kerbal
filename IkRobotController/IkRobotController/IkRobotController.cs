using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using KSP.IO;

// FELADATOK:
// - Plugin run only flight mode ???
// - Plugin start only if push button and exist on arm
// - Check module part is PDGF ???
// - RobotArm with dynamic baseposition ??? - dynamic dumping servo structure ???
// - Initialize from notbase position (any theta != 0)
// - A-Lee is ModulePart ???
// - Separate ServoStructure data from all servo data
// - Save/Load ServoStructure To/From config file in EditorMode
// - Get Axis information from arm structure

namespace IkRobotController
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class IkRobotController : PartModule
    {
        private static int onSaveCounter = 0;
        private static int onLoadCounter = 0;

        protected static int NextWindowID = 110;
        protected int WindowID = 0;
        protected int MsgWindowID = 0;

        private bool IsInitedModule = false;
        private bool IsSaveServoStructure = false;

        private string msgWindowTitle = "";
        private string msgText = "";

        private Rect windowRectangle;
        private Rect msgWindowRectangle;
        private Vector2 windowPosition = new Vector2(10, 10);
        private Vector2 windowSize = new Vector2(310, 600);
        private Vector2 msgWindowPosition = new Vector2(100, 100);
        private Vector2 msgWindowSize = new Vector2(200, 100);
        private Rect inputRect;
        private ConfigNode nodeInner;
        private bool IK_active = false;
        private bool IKButton_active = false;
        private bool servotransform = false;
        private bool zeroThetaIterate = false;
        private bool clearThetasBool = false;
        private bool baseStateBool = false;
        private bool actualStateBool = false;
        private bool pdgfStateBool = false;
        private bool ircWindowActive = false;
        private bool msgWindowActive = false;
        private float Yoffset = 0;
        private float servoSpeed = 0.25f;
        private float transStep = 0.25f;
        private float rotStep = 0.25f;

        private string iterateString = "500";
        private string samplingAngleString = "0.02";
        private string DistanceThresholdString = "0.01";
        private string AngleThresholdString = "0.5";
        private string MaxPosErrString = "10";
        private string MaxOriErrString = "180";
        private string[] thetaString = { "0", "0", "0", "0", "0", "0", "0" };

        private Rect diagramRectangle;
        private List<float> iterateData;
        private List<float> sampleAngleData;
        private List<List<float>> data;

        public class VectorSM
        {
            public Vector3 Translation;
            public Quaternion Rotation;

            public VectorSM()
            {
                Translation = new Vector3();
                Rotation = Quaternion.identity;
            }
        }

        public class LocationRotationDiff
        {
            public float LocationDiff;
            public float RotationDiff;

            public LocationRotationDiff()
            {
            }
        }

        public class Buttons
        {
            public Vector3 Translation;
            public Quaternion Rotation;

            public Buttons()
            {
                Translation = new Vector3();
                Rotation = Quaternion.identity;
            }
        }

        public class FKparams
        {
            public Quaternion PartRotation;
            public Vector3 ParentOffset;
            public Vector3 Axis;
            //public Vector3 Position;
            public Quaternion Rotation;

            public FKparams()
            {
            }

            //public void ConfigWriteParams(PluginConfiguration config, int servoNumber, FKparams fkParams)
            //{
            //    config.SetValue(servoNumber.ToString() + "-Servo-Params-ParentOffset", fkParams.ParentOffset);
            //    config.SetValue(servoNumber.ToString() + "-Servo-Params-Axis", fkParams.Axis);
            //    config.SetValue(servoNumber.ToString() + "-Servo-Params-Position", fkParams.Position);
            //    config.SetValue(servoNumber.ToString() + "-Servo-Params-Rotation", fkParams.Rotation);

            //    config.save();

            //    Debug.Log(string.Format("[TRF] {0} - ConfigWriteParams( " + servoNumber.ToString() + "-Servo-Params )", 50));

            //    //config.SetValue("controlWindowPosition", _controlWindowPosition);
            //    //config.SetValue("editorWindowPosition", _editorWindowPosition);
            //    //config.SetValue("editorWindowSize", _editorWindowSize);
            //    //config.SetValue("uiSettingsWindowPosition", _settingsWindowPosition);
            //    //config.SetValue("presetsWindowPosition", _presetsWindowPosition);
            //    //config.SetValue("UIAlphaValue", (double)_UIAlphaValue);
            //    //config.SetValue("UIScaleValue", (double)_UIScaleValue);
            //    //config.SetValue("useEC", UseElectricCharge);
            //    //config.SetValue("useBlizzyToolbar", useBlizzyToolbar);

            //    //config.save();
            //}

            //public FKparams ConfigReadParams(PluginConfiguration config, int servoNumber)
            //{
            //    FKparams fkParams = new FKparams();

            //    config.load();

            //    fkParams.ParentOffset = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-ParentOffset");
            //    fkParams.Axis = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-Axis");
            //    fkParams.Position = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-Position");
            //    fkParams.Rotation = config.GetValue<Quaternion>(servoNumber.ToString() + "-Servo-Params-Rotation");

            //    return fkParams;

            //    //config.load();

            //    //_controlWindowPosition = config.GetValue<Vector3>("controlWindowPosition");
            //    //_editorWindowPosition = config.GetValue<Vector3>("editorWindowPosition");
            //    //_editorWindowSize = config.GetValue<Vector2>("editorWindowSize");
            //    //_settingsWindowPosition = config.GetValue<Vector3>("uiSettingsWindowPosition");
            //    //_presetsWindowPosition = config.GetValue<Vector3>("presetsWindowPosition");

            //    //_UIAlphaValue = (float)config.GetValue<double>("UIAlphaValue", 0.8);
            //    //_UIScaleValue = (float)config.GetValue<double>("UIScaleValue", 1.0);
            //    //UseElectricCharge = config.GetValue<bool>("useEC", true);
            //    //useBlizzyToolbar = config.GetValue<bool>("useBlizzyToolbar", false);
            //}
        }

        public class IKservo
        {
            public Part part;
            public IRWrapper.IServo iservo;
            //public Vector3 ParentOffset;
            //public Vector3 Position;
            //public Quaternion Rotation;
            //public Quaternion localRotation;
            //public Vector3 Axis;
            public Transform ServoTransform;
            public string name;
            public float MinAngle = -270f;
            public float MaxAngle = 270f;

            public FKparams fkParams;

            //public Vector3 StartRotOffset;

            public IKservo(Part part, IRWrapper.IServo iservo)
            {
                this.part = part;
                this.iservo = iservo;
                this.fkParams = new FKparams();

            }

            public IKservo(Part part)
            {
                this.part = part;
                this.fkParams = new FKparams();
            }

            public IKservo(FKparams fkParams)
            {
                this.fkParams = fkParams;
            }
        }

        Buttons buttons;
        List<IRWrapper.IServo> allServos;
        List<IKservo> AllIkServo;
        List<IKservo> SortIkServo;
        List<IKservo> FKparamsIkServo;
        List<Part> listOfChildrenPart;
        VectorSM FKvector = new VectorSM();
        VectorSM baseState = new VectorSM();
        GameObject[] servoGimbal = new GameObject[8];
        Part lastPart;

        GameObject hoverObject;
        Transform dockingNodeObject;

        Mode mode;
        int limitDepth = 2;

        private enum Mode
        {
            PART,
            UI,
            OBJECT
        }

        string[] JointList = { "TRF.CA2.ARoll", "TRF.CA2.AYaw", "TRF.CA2.APitch", "TRF.CA2.CElbow", "TRF.CA2.BPitch", "TRF.CA2.BYaw", "TRF.CA2.BRoll", "TRF.CA2.LEE.wCam" };
        Vector3[] JointsAxis = { new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 0) };
        // !!! calculate value of JointsRealAxis !!!
        // PDGF Axis list
        //Vector3[] JointsRealAxis = { new Vector3(0, -1, 0), new Vector3(-1, 0, 0), new Vector3(0, 0, 1), new Vector3(0, 0, -1), new Vector3(0, 0, -1), new Vector3(-1, 0, 0), new Vector3(0, -1, 0), new Vector3(0, 0, 0) };
        Vector3[] JointsBaseAxis = { new Vector3(0, -1, 0), new Vector3(-1, 0, 0), new Vector3(0, 0, 1), new Vector3(0, 0, -1), new Vector3(0, 0, -1), new Vector3(-1, 0, 0), new Vector3(0, -1, 0), new Vector3(0, 0, 0) };
        // LEE Axis list
        Vector3[] JointsRealAxis = { new Vector3(0, 1, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1), new Vector3(0, 0, -1), new Vector3(0, 0, -1), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 0) };
        public float[] theta = new float[8];

        public bool[] blocking = new bool[8];
        public float distance;
        public float angle;
        public bool Success = false;
        public bool IKsuccess = false;
        public bool Bug = true;

        public Vector3 globalPosition;
        public Quaternion globalQuaternion;

        public int iterateNumber;
        public float LearningRatePos;
        public float LearningRateOri;
        public float SamplingAngle;

        public float DistanceThreshold;
        public float Distance;
        public float AngleThreshold;
        public float Angle;

        public float MaxPosErr;
        public float MaxOriErr;

        public float DinLearningRatePos;
        public float DinLearningRateOri;

        public int failedIterateCycle = 0;
        public int iterateThreshold = 10;

        private Vector3 basePosition;
        private Quaternion baseQuaternion;

        private Vector3 partPosition;
        private Quaternion partRotation;

        private Vector3 prevPartPosition;
        private Quaternion prevPartRotation;

        #region mapping servo structure
        private void CreateFKparams()
        {
            FKparamsIkServo = new List<IKservo>();

            for (int i = 0; i < 8; i++)
            {
                FKparamsIkServo.Add(new IKservo(new FKparams()));
            }
        }

        private bool CheckExistArm()
        {
            for (int i = 0; i < JointList.Length; i++)
                if (part.FindChildPart(JointList[i], true) == null)
                    return false;

            return true;
        }

        private void GetAllChildServo()
        {
            // Get all servo
            foreach (IRWrapper.IControlGroup group in IRWrapper.IRController.ServoGroups)
            {
                allServos.AddRange(group.Servos);
            }

            //foreach (IRWrapper.IServo element in allServos)
            //{
            //    Debug.Log(string.Format("[TRF] {0} - allServos.Add( " + element.HostPart.name + " )", 22));
            //}

            // Scanning all servos
            foreach (IRWrapper.IServo iservo in allServos)
            {
                //Debug.Log(string.Format("[TRF] {0} - iservo.HostPart.name( " + iservo.HostPart.name + " )", 22));
                foreach (Part element in part.FindChildParts<Part>(true))
                {
                    //Debug.Log(string.Format("[TRF] {0} - FindChildParts<Part>( " + element.name + " )", 22));
                    // if childparts contain servo and not hinge then add to IKservos
                    if ((iservo.HostPart.name == element.name) && !iservo.HostPart.name.Contains("Hinge"))
                    {
                        AllIkServo.Add(new IKservo(element, iservo));
                        Debug.Log(string.Format("[TRF] {0} - AllIkServo.Add( " + element.name + " )", 22));
                    }
                }
            }

            Debug.Log(string.Format("[TRF] {0} - END GetAllChildServo()", 22));
        }

        private void RemoveHingeServo()
        {
            foreach (IRWrapper.IServo iservo in allServos)
            {
                if (!iservo.HostPart.name.Contains("Hinge"))
                {
                    AllIkServo.Add(new IKservo(part, iservo));
                }
            }

            Debug.Log(string.Format("[TRF] {0} - END RemoveHingeServo()", 23));
        }

        //private void RemoveFalseServo()
        //{
        //    foreach (IRWrapper.IServo iservo in allServos)
        //    {
        //        if (Array.IndexOf(JointList, iservo.HostPart.name) > -1)
        //        {
        //            AllIkServo.Add(new IKservo(part, iservo));
        //            Debug.Log(string.Format("[TRF] {0} - AllIkServo.Add( " + part.name + " )", 23));

        //        }
        //    }

        //    Debug.Log(string.Format("[TRF] {0} - END RemoveFalseServo()", 23));
        //}

        private void SortIKservos()
        {
            for (int i = 0; i < JointList.Length; i++)
            {
                foreach (IKservo ikServo in AllIkServo)
                {
                    if (ikServo.part.name == JointList[i])
                    {
                        SortIkServo.Add(ikServo);
                        Debug.Log(string.Format("[TRF] {0} - ikServo " + ikServo.part.name, 24));
                    }
                }
            }

            Debug.Log(string.Format("[TRF] {0} - END SortIKservos()", 24));
        }

        private void DumpServoStructure(List<IKservo> IkServos)
        {
            Vector3 StartPos = part.transform.position;
            Quaternion StartRot = part.transform.rotation;
            int i = 0;

            // dumping servo structure
            foreach (IKservo ikServo in IkServos)
            {
                // transform of servo
                ikServo.ServoTransform = ikServo.part.transform;
                // orientation of main part
                ikServo.fkParams.PartRotation = part.transform.rotation;
                // position offset for parentpart
                ikServo.fkParams.ParentOffset = Quaternion.Inverse(StartRot) * (ikServo.ServoTransform.position - StartPos);
                // axis of servo
                //ikServo.fkParams.Axis = part.transform.rotation * (part.transform.localRotation * JointsRealAxis[i]);
                ikServo.fkParams.Axis = part.transform.rotation * JointsRealAxis[i];
                //ikServo.fkParams.Axis = Quaternion.Inverse(part.transform.rotation) * JointsBaseAxis[i];
                // global rotation of servo
                ikServo.fkParams.Rotation = ikServo.ServoTransform.rotation;
                // global position of servo
                //ikServo.fkParams.Position = ikServo.ServoTransform.position;
                //// Parent's global position of servo
                //StartPos = ikServo.fkParams.Position;
                StartPos = ikServo.ServoTransform.position;
                // Parent's global rotation of servo
                StartRot = ikServo.fkParams.Rotation;

                Debug.Log(string.Format("[TRF] {0} - ikServo " + ikServo.part.name, 40));
                Debug.Log(string.Format("[TRF] {0} - ParentOffset " + VectorToString(ikServo.fkParams.ParentOffset, "0.00"), 42));
                //Debug.Log(string.Format("[TRF] {0} - Axis " + VectorToString(ikServo.fkParams.Axis, "0.0") + "  " + VectorToString(part.transform.rotation * ikServo.fkParams.Axis, "0.0"), 40));
                Debug.Log(string.Format("[TRF] {0} - Axis " + VectorToString(ikServo.fkParams.Axis, "0.0"), 40));
                // Debug.Log(string.Format("[TRF] {0} - Axis " + VectorToString(JointsRealAxis[i], "0.0") + "  " + VectorToString(part.transform.rotation * JointsRealAxis[i], "0.0"), 40));
                //Debug.Log(string.Format("[TRF] {0} - Position " + VectorToString(ikServo.fkParams.Position, "0.00"), 42));
                Debug.Log(string.Format("[TRF] {0} - Rotation " + QuaternionToString(ikServo.fkParams.Rotation, "0.00"), 42));
                //Debug.Log(string.Format("[TRF] {0} - Rotation " + QuaternionToString(Quaternion.Inverse(part.transform.rotation) * ikServo.fkParams.Rotation, "0.00"), 42));

                i++;
            }

            Debug.Log(string.Format("[TRF] {0} - END DumpServoStructure()", 42));

            Debug.Log(string.Format("[TRF] {0} - part.transform.rotation " + QuaternionToString(part.transform.rotation, "0.00"), 42));
        }
        #endregion mapping servo structure

        // Use this for initialization
        public void Start()
        {
            IsInitedModule = false;
            msgWindowRectangle = new Rect(msgWindowPosition.x, msgWindowPosition.y, msgWindowSize.x, msgWindowSize.y);

            if (HighLogic.LoadedSceneIsFlight)
            {
                if (FKparamsIkServo == null)
                    CreateFKparams();

                WindowID = NextWindowID;
                NextWindowID++;
                MsgWindowID = NextWindowID;
                NextWindowID++;

                if (part == null)
                    Debug.Log(string.Format("[TRF{1}] {0} Start() part == null", 0, WindowID));
                else
                    Debug.Log(string.Format("[TRF{1}] {0} Start() part.name = " + part.name, 0, WindowID));

                diagramRectangle = new Rect(10, 525, 290, 70);

                //// Set base angle of joints = 0 degree
                //theta[0] = theta[1] = theta[2] = theta[3] = theta[4] = theta[5] = theta[6] = theta[7] = 0.0f;

                blocking[0] = blocking[1] = blocking[2] = blocking[3] = blocking[4] = blocking[5] = blocking[6] = blocking[7] = false;
                // Init Buttons
                buttons = new Buttons();

                // Set start position and  start orientation of EndEffector
                globalPosition = new Vector3(0f, 0f, 0f);
                globalQuaternion = Quaternion.Euler(0f, 0f, 0f);

                // check exist arm on part
                if (!CheckExistArm())
                {
                    Debug.Log(string.Format("[TRF] {0} Don't Exist Arm on PART", 0));
                    return;
                }
                else
                {
                    Debug.Log(string.Format("[TRF] {0} Exist Arm on PART", 0));
                }

                Debug.Log(string.Format("[TRF] {0} IRWrapper.InitWrapper() START", 1));
                IRWrapper.InitWrapper();
                Debug.Log(string.Format("[TRF] {0} IRWrapper.InitWrapper() END", 2));

                if (IRWrapper.APIReady)
                {
                    Debug.Log(string.Format("[TRF] {0} - IRWrapper.APIReady", 21));
                    AllIkServo = new List<IKservo>();
                    allServos = new List<IRWrapper.IServo>();
                    SortIkServo = new List<IKservo>();
                    Debug.Log(string.Format("[TRF] {0} - Inited ServoLists", 21));

                    GetAllChildServo();
                    Debug.Log(string.Format("[TRF] {0} - Get all child servo ({1})", 22, allServos.Count));

                    if (allServos.Count >= JointList.Length)
                    {
                        Debug.Log(string.Format("[TRF] {0} - AllIkServo ({1})", 23, AllIkServo.Count));

                        SortIKservos();

                        // Get last part
                        listOfChildrenPart = new List<Part>();
                        GetChildPartRecursive(part);
                        int elementCount = listOfChildrenPart.Count();
                        lastPart = listOfChildrenPart[elementCount - 1];

                        Debug.Log(string.Format("[TRF] {0} - lastPart " + lastPart.name, 40));

                        // Add EndEffector/LEE/ (last part) to SortIkServo - problem -> if LEE hold other element that will last part
                        SortIkServo.Add(new IKservo(lastPart));
                        Debug.Log(string.Format("[TRF] {0} - END - SortIkServo.Add(new IKservo(lastPart)", 41));

                        // if have all IKservo than calculate IK parameters of IKservos 
                        if (SortIkServo.Count == JointList.Length)
                        {
                            partPosition = part.transform.position;
                            partRotation = part.transform.rotation;

                            // dumping servo structure
                            DumpServoStructure(SortIkServo);

                            //Debug.Log(string.Format("[TRF] {0} - START - GetFKParamsFromBuffer()", 42));
                            GetFKParamsFromBuffer();
                            //Debug.Log(string.Format("[TRF] {0} - END - GetFKParamsFromBuffer()", 43));

                        }
                    }
                }
                else
                    return;

                Debug.Log(string.Format("[TRF] {0} - START - globalPosition&Quaternion", 44));
                // set position of last part for global position
                globalPosition = SortIkServo[SortIkServo.Count - 1].ServoTransform.position;
                globalQuaternion = SortIkServo[SortIkServo.Count - 1].ServoTransform.rotation;
                Debug.Log(string.Format("[TRF] {0} - START - globalPosition&Quaternion", 45));

                baseState.Translation = globalPosition;
                baseState.Rotation = globalQuaternion;

                // Store base position and base orientation
                basePosition = globalPosition;
                baseQuaternion = globalQuaternion;

                Debug.Log(string.Format("[TRF] {0} - START - servoGimbal[i]", 46));
                for (int i = 0; i < SortIkServo.Count; i++)
                    servoGimbal[i] = new GameObject();
                Debug.Log(string.Format("[TRF] {0} - END - servoGimbal[i]", 47));

                Debug.Log(string.Format("[TRF] {0} - START - EndDeclaration", 48));
                windowRectangle = new Rect(windowPosition.x, windowPosition.y, windowSize.x, windowSize.y);
                iterateData = new List<float>();
                sampleAngleData = new List<float>();
                data = new List<List<float>>();
                Debug.Log(string.Format("[TRF] {0} - END - EndDeclaration", 49));

                Debug.Log(string.Format("[TRF] {0} end of Start() WindowID: {1}", 400, WindowID));

                // Get angle of Joints
                for (int j = 0; j < SortIkServo.Count - 1; j++)
                    theta[j] = SortIkServo[j].iservo.Position;
                //if (!double.IsNaN(theta[j]))
                //{
                //    // blocking fullrotate of servo
                //    if (theta[j] > 180f)
                //        servoTheta = theta[j] - 360f;
                //    else if (theta[j] < -180f)
                //        servoTheta = theta[j] + 360f;
                //    else
                //        servoTheta = theta[j];
                //}

                prevPartPosition = part.transform.position;
                prevPartRotation = part.transform.rotation;

                IsInitedModule = true;
            }
        }

        // Update is called once per frame
        public void Update()
        {
            //if (part.transform.InverseTransformPoint(prevPartPosition).magnitude > 0.1f || (Quaternion.Inverse(prevPartRotation) * part.transform.rotation).eulerAngles.magnitude > 0.1f)
            //{
            //    // global orientation
            //    globalQuaternion = globalQuaternion * (Quaternion.Inverse(prevPartRotation) * part.transform.rotation);
            //    // global position
            //    globalPosition = globalPosition + globalQuaternion * part.transform.InverseTransformPoint(prevPartPosition);
            //}
            //prevPartPosition = part.transform.position;
            //prevPartRotation = part.transform.rotation;

            // Dump Button enable / disable
            if (HighLogic.LoadedSceneIsFlight && (theta[0] == 0 && theta[1] == 0 && theta[2] == 0 && theta[3] == 0 && theta[4] == 0 && theta[5] == 0 && theta[6] == 0 && theta[7] == 0))
            {
                Events["DumpServoStructureEvent"].active = true;
            }
            else
            {
                Events["DumpServoStructureEvent"].active = false;
            }

            if (ircWindowActive && HighLogic.LoadedSceneIsFlight && IsInitedModule)
            {
                //Debug.Log(string.Format("[TRF] {0} START Update()", 401));

                // Set Aim Object (PDGFx)
                GameObject mouseObject = CheckForObjectUnderCursor();
                //if (mouseObject != null)
                //    Debug.Log(string.Format("[TRF] {0} mouseObject " + mouseObject.name, 420));
                //else
                //    Debug.Log(string.Format("[TRF] {0} mouseObject null", 420));
                //string info = mouseObject ? mouseObject.name : "Nothing";
                //Debug.Log(string.Format("[TRF] {0} - info.text " + info, 403));
                bool modPressed = Input.GetKey(KeyCode.LeftAlt);
                if (modPressed && (mouseObject != null))
                {
                    //Debug.Log(string.Format("[TRF] {0} - modPressed active", 500));
                    hoverObject = mouseObject;
                    Debug.Log(string.Format("[TRF] {0} - hoverObject " + hoverObject.name, 501));

                    foreach (Transform element in hoverObject.GetComponentsInChildren<Transform>())
                    {
                        if (element.name == "dockingNode")
                        {
                            dockingNodeObject = element;
                            Debug.Log(string.Format("[TRF] {0} - dockingNodeObject = " + dockingNodeObject.name, 502));
                            break;
                        }
                    }
                    //    currentDisplayedObject = GetRootObject(hoverObject);
                    //    Debug.Log(string.Format("[TRF] {0} - currentDisplayedObject " + currentDisplayedObject.name, 502));
                }

                // set base state
                if (baseStateBool)
                {
                    globalPosition = basePosition;
                    globalQuaternion = baseQuaternion;

                    for (int i = 0; i < 8; i++)
                        theta[i] = 0f;

                    IK_active = false;

                    // implement angle of joints
                    Bug = !ImplementServoRotation(SortIkServo, theta, servoSpeed);
                }

                // set actual state
                if (actualStateBool)
                {
                    FKvector = ForwardKinematics(theta, SortIkServo);
                    globalPosition = FKvector.Translation;
                    globalQuaternion = FKvector.Rotation;

                    IK_active = false;
                }

                // set pdgf state - set aim object -> deactivate IK function
                //if (pdgfStateBool && (hoverObject != null))
                //{
                //    globalQuaternion = hoverObject.transform.rotation * Quaternion.Euler(180f, -90f, 0f);
                //    globalPosition = hoverObject.transform.position + (globalQuaternion * new Vector3(0f, -0.9f, 0f));
                //    IK_active = false;
                //}
                if (pdgfStateBool && (dockingNodeObject != null))
                {
                    globalQuaternion = dockingNodeObject.rotation * Quaternion.Euler(180f, -90f, 90f);
                    globalPosition = dockingNodeObject.position + (globalQuaternion * new Vector3(0f, -0.9f, 0f));
                    IK_active = false;
                }

                // detect button activity
                IKButton_active = !ButtonsIsReleased();

                // if rotate only servo of LEE then not IK only rotate servo to direct
                if (buttons.Rotation == Quaternion.Euler(0f, 0.5f * rotStep, 0f) || buttons.Rotation == Quaternion.Euler(0f, -0.5f * rotStep, 0f))
                {
                    theta[6] += (360f - buttons.Rotation.eulerAngles.y) > 180f ? buttons.Rotation.eulerAngles.y : (buttons.Rotation.eulerAngles.y - 360f);
                    // implement angle of joints
                    Bug = !ImplementServoRotation(SortIkServo, theta, servoSpeed);
                }

                // calculate global position of aim from buttons command
                // global orientation
                globalQuaternion = globalQuaternion * buttons.Rotation;
                // global position
                globalPosition = globalPosition + globalQuaternion * buttons.Translation;

                // clear buttons.Translation & .Rotation
                ButtonsReleased();

                #region Active Inverse Kinematics
                if (IK_active || IKButton_active)
                {
                    // calculate position- and orientation different
                    distance = Vector3.Distance(SortIkServo[SortIkServo.Count - 1].ServoTransform.position, globalPosition);
                    angle = Quaternion.Angle(Quaternion.Euler(SortIkServo[SortIkServo.Count - 2].ServoTransform.rotation.eulerAngles), globalQuaternion);

                    // if differents over the threshold then calculate IK
                    if (distance > DistanceThreshold || angle > AngleThreshold)
                    {
                        Success = false;

                        // start iterate from zero thetas
                        if (zeroThetaIterate && IKsuccess)
                        {
                            for (int i = 0; i < 8; i++)
                                theta[i] = 0f;
                        }
                        // clear thetas
                        if (clearThetasBool)
                            for (int i = 0; i < 8; i++)
                                theta[i] = 0f;

                        // iterate angle of joints
                        int j;
                        for (j = 0; j < iterateNumber; j++)
                        {
                            IKsuccess = InverseKinematics(globalPosition, globalQuaternion, theta);
                            if (IKsuccess)
                                break;
                        }

                        iterateData.Add((float)j / (float)iterateNumber * (diagramRectangle.height - 20f));
                        if (iterateData.Count > (int)(diagramRectangle.width - 10f))
                            iterateData.RemoveAt(0);

                        if (j == iterateNumber)
                        {
                            failedIterateCycle++;
                            SamplingAngle = 0.005f;
                        }
                        else
                        {
                            failedIterateCycle = 0;
                            if (j > (iterateNumber / 2))
                            {
                                SamplingAngle = 0.01f;
                            }
                            else
                            {
                                SamplingAngle = 0.02f;
                            }
                        }
                        samplingAngleString = SamplingAngle.ToString();

                        sampleAngleData.Add(SamplingAngle / 0.02f * (diagramRectangle.height - 30f));
                        if (sampleAngleData.Count > (int)(diagramRectangle.width - 10f))
                            sampleAngleData.RemoveAt(0);

                        if (failedIterateCycle > iterateThreshold)
                        {
                            failedIterateCycle = 0;

                            for (int i = 0; i < 8; i++)
                                theta[i] = 0f;
                        }

                        // implement angle of joints
                        Bug = !ImplementServoRotation(SortIkServo, theta, servoSpeed);
                    }
                    else
                    {
                        Success = true;
                        // if IK success then turn off IK activity or switchable ???
                        IK_active = false;
                    }
                    //Debug.Log(string.Format("[TRF] {0} - Update() END ", 107));
                }
                #endregion Active Inverse Kinematics
                #region Inactive Inverse Kinematics
                else
                {
                    // clear buttons.Translation & .Rotation
                    ButtonsReleased();
                    //Debug.Log(string.Format("[TRF] {0} - ButtonsReleased()", 100));
                    FKvector = ForwardKinematics(theta, SortIkServo);
                    //Debug.Log(string.Format("[TRF] {0} - ForwardKinematics(..)", 101));

                    // implement angle of joints
                    Bug = !ImplementServoRotation(SortIkServo, theta, servoSpeed);

                    for (int i = 0; i < 7; i++)
                    {
                        if (blocking[i])
                        {
                            SortIkServo[i].MaxAngle = 0f;
                            SortIkServo[i].MinAngle = 0f;
                        }
                        else
                        {
                            SortIkServo[i].MaxAngle = 270f;
                            SortIkServo[i].MinAngle = -270f;
                        }
                    }
                }
                #endregion Inactive Inverse Kinematics

                // Refresh data List of LineDiagram
                if (data != null)
                {
                    data.Clear();
                    data.Add(iterateData);
                    data.Add(sampleAngleData);
                }
            }
        }

        #region get part transform by mouse
        private GameObject CheckForObjectUnderCursor()
        {
            //if (EventSystem.current.IsPointerOverGameObject())
            //{
            //    return null;
            //}


            //1000000000000000000101

            if (mode == Mode.UI)
            {
                var pointer = new PointerEventData(EventSystem.current);
                pointer.position = Input.mousePosition;

                var raycastResults = new List<RaycastResult>();
                EventSystem.current.RaycastAll(pointer, raycastResults);

                if (raycastResults.Count == 0)
                {
                    //print("Nothing");
                    return null;
                }
                return raycastResults[0].gameObject;
            }

            if (mode == Mode.PART)
            {
                //if (Mouse.HoveredPart ? Mouse.HoveredPart.gameObject : null != null)
                //{
                //    Debug.Log(string.Format("[TRF] {0} Mode.PART " + Mouse.HoveredPart.gameObject.ToString(), 410));
                //}
                //else
                //{
                //    Debug.Log(string.Format("[TRF] {0} Mode.PART null", 410));
                //}

                return Mouse.HoveredPart ? Mouse.HoveredPart.gameObject : null;
            }

            if (mode == Mode.OBJECT)
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                //int layerMask = ~LayerMask.NameToLayer("UI");
                int layerMask = ~0;

                RaycastHit hit;
                if (!Physics.Raycast(ray, out hit, Mathf.Infinity, layerMask))
                {
                    return null;
                }
                return hit.collider.gameObject;
            }

            return null;
        }

        private GameObject GetRootObject(GameObject leaf)
        {
            if (leaf == null)
                return null;

            if (mode == Mode.UI)
            {
                int d = 0;
                while (leaf.transform.parent && !leaf.transform.parent.gameObject.GetComponent<Canvas>() && d < limitDepth)
                {
                    leaf = leaf.transform.parent.gameObject;
                    d++;
                }
                return leaf;
            }

            if (mode == Mode.PART)
            {
                return leaf;
            }

            if (mode == Mode.OBJECT)
            {
                int d = 0;
                while (leaf.transform.parent && d < limitDepth)
                {
                    leaf = leaf.transform.parent.gameObject;
                    d++;
                }
                return leaf;
            }

            return null;
        }
        #endregion get part transform by mouse

        #region KSPEvents
        [KSPEvent(guiActive = true, guiName = "Dump ServoStructure")]
        public void DumpServoStructureEvent()
        {
            // Genarate servo structure and save
            if (FKparamsIkServo == null)
                CreateFKparams();

            if (part == null)
                Debug.Log(string.Format("[TRF{1}] {0} START Update() part == null", 0, WindowID));
            else
                Debug.Log(string.Format("[TRF{1}] {0} START Update() part.name = " + part.name, 0, WindowID));

            // check exist arm on part
            if (!CheckExistArm())
            {
                Debug.Log(string.Format("[TRF] {0} Don't Exist Arm on PART", 0));
                return;
            }
            else
            {
                Debug.Log(string.Format("[TRF] {0} Exist Arm on PART", 0));
            }

            //Debug.Log(string.Format("[TRF] {0} IRWrapper.InitWrapper() START", 1));
            //IRWrapper.InitWrapper();
            //Debug.Log(string.Format("[TRF] {0} IRWrapper.InitWrapper() END", 2));

            if (IRWrapper.APIReady)
            {
                Debug.Log(string.Format("[TRF] {0} - IRWrapper.APIReady", 21));
                AllIkServo = new List<IKservo>();
                allServos = new List<IRWrapper.IServo>();
                SortIkServo = new List<IKservo>();
                Debug.Log(string.Format("[TRF] {0} - Inited ServoLists", 21));

                GetAllChildServo();
                Debug.Log(string.Format("[TRF] {0} - Get all child servo ({1})", 22, allServos.Count));

                if (allServos.Count >= JointList.Length)
                {
                    Debug.Log(string.Format("[TRF] {0} - AllIkServo ({1})", 23, AllIkServo.Count));

                    SortIKservos();

                    // Get last part
                    listOfChildrenPart = new List<Part>();
                    GetChildPartRecursive(part);
                    int elementCount = listOfChildrenPart.Count();
                    lastPart = listOfChildrenPart[elementCount - 1];

                    Debug.Log(string.Format("[TRF] {0} - lastPart " + lastPart.name, 40));

                    // Add EndEffector/LEE/ (last part) to SortIkServo - problem -> if LEE hold other element that will last part
                    SortIkServo.Add(new IKservo(lastPart));

                    // if have all IKservo than calculate IK parameters of IKservos 
                    if (SortIkServo.Count == JointList.Length)
                    {
                        partPosition = part.transform.position;
                        partRotation = part.transform.rotation;

                        // dumping servo structure
                        DumpServoStructure(SortIkServo);

                        SetFKParamsToBuffer();
                        //if (nodeInner != null)
                        //{
                        OnSave(null);
                        Debug.Log(string.Format("[TRF] {0} Servo Structure Dumped", 50));
                        MsgWindow(new Vector2(1920f / 2f, 1080f / 2f), "Servo dumping message", "Servo Structure Dumped");
                        //}
                        //else
                        //{
                        //    MsgWindow(new Vector2(1920f / 2f, 1080f / 2f), "Servo dumping message", "Servo Structure Not Dumped");
                        //    return;
                        //}
                    }
                    else
                        MsgWindow(new Vector2(1920f / 2f, 1080f / 2f), "Servo dumping message", "Servo Structure Not Dumped");

                }
            }
            else
                return;
        }

        [KSPEvent(guiActive = true, guiName = "Turn on IKRC")]
        public void TurnOnIKRCEvent()
        {
            if (IsInitedModule)
            {
                GetWinParamsFromBuffer();
                // This will hide the TurnOnIKRCEvent event, and show the TurnOffIKRCEvene event.
                Events["TurnOnIKRCEvent"].active = false;
                Events["TurnOffIKRCEvent"].active = true;
                ircWindowActive = true;
            }
        }

        [KSPEvent(guiActive = true, guiName = "Turn off IKRC", active = false)]
        public void TurnOffIKRCEvent()
        {
            // This will hide the TurnOffIKRCEven event, and show the TurnOnIKRCEvent event.
            Events["TurnOnIKRCEvent"].active = true;
            Events["TurnOffIKRCEvent"].active = false;
            ircWindowActive = false;
            //Debug.Log("[TRF] TurnOffIKRCEvent()}");
            SetWinParamsToBuffer();
            OnSave(nodeInner);
        }
        #endregion KSPEvents

        #region OnMethods
        public void OnRenderObject()
        {
            if (ircWindowActive)
            {
                if (servotransform)
                {
                    //Debug.Log(string.Format("[TRF] {0} - !IK_active ", 102));
                    for (int i = 0; i < (servoGimbal.Length - 1); i++)
                    {
                        DrawTools.DrawTransform(servoGimbal[i].transform, 0.3f);
                        //Debug.Log(string.Format("[TRF] {0} - DrawTransform[{1}]", 103, i));
                    }

                    //if (hoverObject != null)
                    //    DrawTools.DrawTransform(hoverObject.transform, 0.3f);

                    if (dockingNodeObject != null)
                        DrawTools.DrawTransform(dockingNodeObject, 0.3f);

                    DrawTools.DrawTransform(SortIkServo[SortIkServo.Count - 1].ServoTransform, 0.3f);
                }
            }
        }

        public override void OnSave(ConfigNode node)
        {
            if ((HighLogic.LoadedSceneIsFlight && IsInitedModule) || HighLogic.LoadedSceneIsEditor)
            {
                PluginConfiguration config = PluginConfiguration.CreateForType<IkRobotController>();

                // save window parameters from buffer to config.xml
                config.SetValue("IRC Window Position", windowPosition);
                Debug.Log(string.Format("[TRF] {0} - OnSave() windowPosition {1}, {2}", 0, windowPosition.x, windowPosition.y));

                // save FK parameters from buffer to config.xml
                if (FKparamsIkServo != null)
                {
                    if (FKparamsIkServo.Count == 8)
                    {
                        int servoNumber = 0;
                        foreach (IKservo ikServo in FKparamsIkServo)
                        {
                            config.SetValue(servoNumber.ToString() + "-Servo-Params-PartRotation", ikServo.fkParams.PartRotation);
                            config.SetValue(servoNumber.ToString() + "-Servo-Params-ParentOffset", ikServo.fkParams.ParentOffset);
                            config.SetValue(servoNumber.ToString() + "-Servo-Params-Axis", ikServo.fkParams.Axis);
                            //config.SetValue(servoNumber.ToString() + "-Servo-Params-Position", ikServo.fkParams.Position);
                            config.SetValue(servoNumber.ToString() + "-Servo-Params-Rotation", ikServo.fkParams.Rotation);

                            servoNumber++;
                        }

                        config.save();

                        Debug.Log(string.Format("[TRF] {0} end of OnSave()", 0));
                    }
                }
            }

            Debug.Log(string.Format("[TRF] - OnSave() " + HighLogic.LoadedScene.ToString()));
            Debug.Log(string.Format("[TRF] - OnSave() {0}", onSaveCounter));
            onSaveCounter++;
        }

        public override void OnLoad(ConfigNode node)
        {
            Debug.Log(string.Format("[TRF] {0} Start of OnLoad()", 0));

            //if (HighLogic.LoadedSceneIsFlight /*|| HighLogic.LoadedSceneIsEditor*/)
            //{
            PluginConfiguration config = PluginConfiguration.CreateForType<IkRobotController>();

            config.load();

            // load window parameters from config.xml to buffer
            windowPosition = config.GetValue<Vector2>("IRC Window Position");
            Debug.Log(string.Format("[TRF] {0} - OnLoad() windowPosition {1}, {2}", 42, windowPosition.x, windowPosition.y));

            if (FKparamsIkServo == null)
                CreateFKparams();

            // load FK parameters from config.xml to buffer
            int servoNumber = 0;
            foreach (IKservo ikServo in FKparamsIkServo)
            {
                ikServo.fkParams.PartRotation = config.GetValue<Quaternion>(servoNumber.ToString() + "-Servo-Params-PartRotation");
                ikServo.fkParams.ParentOffset = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-ParentOffset");
                ikServo.fkParams.Axis = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-Axis");
                //ikServo.fkParams.Position = config.GetValue<Vector3>(servoNumber.ToString() + "-Servo-Params-Position");
                ikServo.fkParams.Rotation = config.GetValue<Quaternion>(servoNumber.ToString() + "-Servo-Params-Rotation");

                Debug.Log(string.Format("[TRF] {0} - OnLoad() " + VectorToString(ikServo.fkParams.ParentOffset, "0.00"), 42));

                servoNumber++;
            }

            GetWinParamsFromBuffer();
            ircWindowActive = false;
            nodeInner = node;
            //}

            Debug.Log(string.Format("[TRF] {0} end of OnLoad()", 0));

            Debug.Log(string.Format("[TRF] - OnLoad() " + HighLogic.LoadedScene.ToString()));
            Debug.Log(string.Format("[TRF] - OnLoad() {0}", onLoadCounter));
            onLoadCounter++;
        }

        private void SetWinParamsToBuffer()
        {
            windowPosition.x = windowRectangle.x;
            windowPosition.y = windowRectangle.y;
        }

        private void GetWinParamsFromBuffer()
        {
            if (windowRectangle != null)
                windowRectangle = new Rect(windowPosition.x, windowPosition.y, windowSize.x, windowSize.y);
        }

        private void SetFKParamsToBuffer()
        {
            int servoNumber = 0;
            foreach (IKservo ikServo in FKparamsIkServo)
            {
                ikServo.fkParams.PartRotation = SortIkServo[servoNumber].fkParams.PartRotation;
                ikServo.fkParams.ParentOffset = SortIkServo[servoNumber].fkParams.ParentOffset;
                ikServo.fkParams.Axis = SortIkServo[servoNumber].fkParams.Axis;
                //ikServo.fkParams.Position = SortIkServo[servoNumber].fkParams.Position;
                ikServo.fkParams.Rotation = SortIkServo[servoNumber].fkParams.Rotation;

                servoNumber++;
            }
        }

        private void GetFKParamsFromBuffer()
        {
            Debug.Log(string.Format("[TRF] {0} - GetFKParamsFromBuffer()", 42));
            Debug.Log(string.Format("[TRF] {0} - part.transform.rotation " + QuaternionToString(part.transform.rotation, "0.00"), 42));
            int servoNumber = 0;
            foreach (IKservo ikServo in FKparamsIkServo)
            {
                SortIkServo[servoNumber].fkParams.PartRotation = ikServo.fkParams.PartRotation;
                SortIkServo[servoNumber].fkParams.ParentOffset = ikServo.fkParams.ParentOffset;
                SortIkServo[servoNumber].fkParams.Axis = ikServo.fkParams.Axis;
                //SortIkServo[servoNumber].fkParams.Axis = part.transform.rotation * ikServo.fkParams.Axis;
                //SortIkServo[servoNumber].fkParams.Position = ikServo.fkParams.Position;
                SortIkServo[servoNumber].fkParams.Rotation = ikServo.fkParams.Rotation;
                //SortIkServo[servoNumber].fkParams.Rotation =  Quaternion.Inverse(ikServo.fkParams.PartRotation) * part.transform.rotation * ikServo.fkParams.Rotation;

                Debug.Log(string.Format("[TRF] {0} - ParentOffset " + VectorToString(SortIkServo[servoNumber].fkParams.ParentOffset, "0.00"), 42));
                //Debug.Log(string.Format("[TRF] {0} - Axis " + VectorToString(ikServo.fkParams.Axis, "0.0") + "  " + VectorToString(part.transform.rotation * ikServo.fkParams.Axis, "0.0"), 40));
                Debug.Log(string.Format("[TRF] {0} - Axis " + VectorToString(ikServo.fkParams.Axis, "0.0"), 40));
                //Debug.Log(string.Format("[TRF] {0} - Position " + VectorToString(SortIkServo[servoNumber].fkParams.Position, "0.00"), 42));
                Debug.Log(string.Format("[TRF] {0} - Rotation " + QuaternionToString(SortIkServo[servoNumber].fkParams.Rotation, "0.00"), 42));

                servoNumber++;
            }

            partRotation = SortIkServo[0].fkParams.PartRotation;
        }

        public void OnDisable()
        {
            //if (ircWindowActive)
            //{
            //    Debug.Log("[TRF] - OnDisable()");
            //    ircWindowActive = false;
            //    OnSave(nodeInner);
            //}
            ircWindowActive = false;
            IK_active = false;
            servotransform = false;
            IsInitedModule = false;
        }

        public void Destroy()
        {
            if (ircWindowActive)
            {
                Debug.Log("[TRF] - Destroy()");
                SetWinParamsToBuffer();
                ircWindowActive = false;
                OnSave(nodeInner);
            }
            IK_active = false;
            servotransform = false;
            IsInitedModule = false;
        }
        #endregion OnMethods

        #region GUI
        public bool ButtonsIsReleased()
        {
            if ((buttons.Translation == new Vector3(0f, 0f, 0f)) && (buttons.Rotation.eulerAngles == new Vector3(0f, 0f, 0f)))
                return true;
            else
                return false;
        }

        public void ButtonsReleased()
        {
            buttons.Translation = new Vector3(0f, 0f, 0f);
            buttons.Rotation = Quaternion.identity;
        }

        public void OnGUI()
        {
            if (ircWindowActive)
            {
                // Draw window
                windowRectangle = GUILayout.Window(WindowID, windowRectangle, OnWindow, "IK Robot Controller");
            }

            if (msgWindowActive)
            {
                // Draw message window
                msgWindowRectangle = GUILayout.Window(MsgWindowID, msgWindowRectangle, OnMsgWindow, msgWindowTitle);
            }
        }

        public void OnWindow(int windowID)
        {
            if (IsInitedModule)
            {
                GUILayout.BeginHorizontal();

                // Translation buttons
                if (GUI.RepeatButton(new Rect(10, 70, 30, 30), "◄"))
                    buttons.Translation.z = 0.025f * transStep;
                if (GUI.RepeatButton(new Rect(80, 70, 30, 30), "►"))
                    buttons.Translation.z = -0.025f * transStep;
                if (GUI.RepeatButton(new Rect(45, 35, 30, 30), "▲"))
                    buttons.Translation.x = -0.025f * transStep;
                if (GUI.RepeatButton(new Rect(45, 105, 30, 30), "▼"))
                    buttons.Translation.x = 0.025f * transStep;
                if (GUI.RepeatButton(new Rect(10, 105, 30, 30), "●"))
                    buttons.Translation.y = -0.025f * transStep;
                if (GUI.RepeatButton(new Rect(80, 35, 30, 30), "•"))
                    buttons.Translation.y = 0.025f * transStep;
                //Debug.Log(string.Format("[TRF] {0} - Translation buttons", 1001));

                // Rotation buttons
                if (GUI.RepeatButton(new Rect(120, 70, 30, 30), "˅ʘ"))
                {
                    buttons.Rotation = Quaternion.Euler(0f, 0.5f * rotStep, 0f);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                if (GUI.RepeatButton(new Rect(190, 70, 30, 30), "ʘ˅"))
                {
                    buttons.Rotation = Quaternion.Euler(0f, -0.5f * rotStep, 0f);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                if (GUI.RepeatButton(new Rect(155, 35, 30, 30), "˄"))
                {
                    buttons.Rotation = Quaternion.Euler(0f, 0f, 0.5f * rotStep);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                if (GUI.RepeatButton(new Rect(155, 105, 30, 30), "˅"))
                {
                    buttons.Rotation = Quaternion.Euler(0f, 0f, -0.5f * rotStep);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                if (GUI.RepeatButton(new Rect(120, 105, 30, 30), "˂"))
                {
                    buttons.Rotation = Quaternion.Euler(0.5f * rotStep, 0f, 0f);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                if (GUI.RepeatButton(new Rect(190, 35, 30, 30), "˃"))
                {
                    buttons.Rotation = Quaternion.Euler(-0.5f * rotStep, 0f, 0f);
                    //Debug.Log(string.Format("[TRF] {0} - buttons.Rotation = " + VectorToString(buttons.Rotation.eulerAngles, "0.00"), 1002));
                }
                //Debug.Log(string.Format("[TRF] {0} - Rotation buttons", 1002));

                // IK active toggle
                IK_active = GUI.Toggle(new Rect(15, 15, 70, 20), IK_active, " IK active");
                // transform of servos toggle
                servotransform = GUI.Toggle(new Rect(100, 15, 80, 20), servotransform, " servoTRF");
                // zeroTheta iterate toggle
                zeroThetaIterate = GUI.Toggle(new Rect(220, 15, 80, 20), zeroThetaIterate, " 0ƟIterate");
                // set aim position and orientation to actual state
                actualStateBool = GUI.Button(new Rect(250, 35, 40, 20), "acSt");
                // clear values of Theta
                clearThetasBool = GUI.Button(new Rect(250, 60, 40, 20), "clrƟ");
                // set base position and orientation
                baseStateBool = GUI.Button(new Rect(250, 85, 40, 20), "0St");
                // set selected position and orientation
                pdgfStateBool = GUI.Button(new Rect(250, 110, 40, 20), "pdgf");
                //Debug.Log(string.Format("[TRF] {0} - IK buttons", 1003));

                // Close window button
                if (GUI.Button(new Rect(290, 3, 17, 15), "x"))
                {
                    ircWindowActive = false;
                    // This will hide the TurnOffIKRCEven event, and show the TurnOnIKRCEvent event.
                    Events["TurnOnIKRCEvent"].active = true;
                    Events["TurnOffIKRCEvent"].active = false;
                    SetWinParamsToBuffer();
                    ircWindowActive = false;
                    Debug.Log("[TRF] Close window button");
                    OnSave(nodeInner);
                }
                //Debug.Log(string.Format("[TRF] {0} - Close window button", 1004));

                // Success value
                GUI.Toggle(new Rect(210, 280, 80, 20), Success, " Success");
                // IK_Success value
                GUI.Toggle(new Rect(210, 300, 90, 20), IKsuccess, " IK Success");
                // Bug value
                GUI.Toggle(new Rect(210, 320, 80, 20), Bug, " Bug");
                //Debug.Log(string.Format("[TRF] {0} - IK toggles", 1005));


                // Theta values
                Yoffset = 0;
                inputRect = new Rect(210, 140, 50, 20);
                for (int i = 0; i < 7; i++)
                {
                    if (IK_active)
                        AddOutputValue(inputRect, "Ɵ[" + i.ToString() + "]", theta[i], 30f);
                    else
                    {
                        thetaString[i] = theta[i].ToString("0.000");
                        AddInputValue(inputRect, thetaString[i], out thetaString[i], "Ɵ[" + i.ToString() + "]", out theta[i], 30f);
                    }
                }
                //Debug.Log(string.Format("[TRF] {0} - Theta values", 1006));

                //Theta blocker
                blocking[0] = GUI.Toggle(new Rect(292, 140, 20, 20), blocking[0], "");
                blocking[1] = GUI.Toggle(new Rect(292, 160, 20, 20), blocking[1], "");
                blocking[2] = GUI.Toggle(new Rect(292, 180, 20, 20), blocking[2], "");
                blocking[3] = GUI.Toggle(new Rect(292, 200, 20, 20), blocking[3], "");
                blocking[4] = GUI.Toggle(new Rect(292, 220, 20, 20), blocking[4], "");
                blocking[5] = GUI.Toggle(new Rect(292, 240, 20, 20), blocking[5], "");
                blocking[6] = GUI.Toggle(new Rect(292, 260, 20, 20), blocking[6], "");
                //Debug.Log(string.Format("[TRF] {0} - Theta blocker", 1007));

                // IK values
                Yoffset = 0;
                inputRect = new Rect(10, 140, 40, 20);
                AddInputValue(inputRect, iterateString, out iterateString, "IK_iterate", out iterateNumber, 140f);
                AddInputValue(inputRect, samplingAngleString, out samplingAngleString, "IK_samplingAngle", out SamplingAngle, 140f);
                AddInputValue(inputRect, DistanceThresholdString, out DistanceThresholdString, "IK_DistanceThreshold", out DistanceThreshold, 140f);
                AddOutputValue(inputRect, "IK_Distance", Distance, 140f);
                AddInputValue(inputRect, AngleThresholdString, out AngleThresholdString, "IK_AngleThreshold", out AngleThreshold, 140f);
                AddOutputValue(inputRect, "IK_Angle", Angle, 140f);
                AddInputValue(inputRect, MaxPosErrString, out MaxPosErrString, "IK_MaxPosErr", out MaxPosErr, 140f);
                AddInputValue(inputRect, MaxOriErrString, out MaxOriErrString, "IK_MaxOriErr", out MaxOriErr, 140f);
                AddOutputValue(inputRect, "IK_DinLearningRatePos", DinLearningRatePos, 140f);
                AddOutputValue(inputRect, "IK_DinLearningRateOri", DinLearningRateOri, 140f);
                AddOutputValue(inputRect, "IK_globalPosition", globalPosition, 140f, 110f);
                AddOutputValue(inputRect, "IK_globalRotation", globalQuaternion.eulerAngles, 140f, 110f);
                //if (SortIkServo == null)
                //    Debug.Log(string.Format("[TRF] {0} - SortIkServo == null", 1008));
                //else if (SortIkServo[SortIkServo.Count - 1] == null)
                //    Debug.Log(string.Format("[TRF] {0} - SortIkServo[SortIkServo.Count - 1] == null", 1008));
                //else if (SortIkServo[SortIkServo.Count - 1].ServoTransform == null)
                //    Debug.Log(string.Format("[TRF] {0} - SortIkServo[SortIkServo.Count - 1].ServoTransform == null", 1008));
                //else if (SortIkServo[SortIkServo.Count - 1].ServoTransform.position == null)
                //    Debug.Log(string.Format("[TRF] {0} - SortIkServo[SortIkServo.Count - 1].ServoTransform.position == null", 1008));
                //else
                //{
                AddOutputValue(inputRect, "EFF_Position", SortIkServo[SortIkServo.Count - 1].ServoTransform.position, 140f, 110f);
                AddOutputValue(inputRect, "EFF_Rosition", SortIkServo[SortIkServo.Count - 1].ServoTransform.rotation.eulerAngles, 140f, 110f);
                //}
                AddOutputValue(inputRect, "distance", distance, 140f);
                AddOutputValue(inputRect, "angle", angle, 140f, 60f);
                //Debug.Log(string.Format("[TRF] {0} - IK values", 1008));

                // slider of servo speed
                float[] speedSliderValues = { 0.0f, 0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
                servoSpeed = FixValuesLogSlider(new Rect(10, 460, 150, 20), 50f, "speed", servoSpeed, speedSliderValues);
                // slider of control button translation's step
                float[] transSliderValues = { 0.0f, 0.0375f, 0.075f, 0.125f, 0.25f, 0.5f, 1.0f, 2.0f };
                transStep = FixValuesLogSlider(new Rect(10, 480, 150, 20), 50f, "TRLstp", transStep, transSliderValues);
                // slider of control button rotation's step
                float[] rotSliderValues = { 0.0f, 0.0375f, 0.075f, 0.125f, 0.25f, 0.5f, 1.0f, 2.0f };
                rotStep = FixValuesLogSlider(new Rect(10, 500, 150, 20), 50f, "ROTstp", rotStep, rotSliderValues);
                //Debug.Log(string.Format("[TRF] {0} - Sliders", 1009));

                // reset position and rotation of endeffector
                if (GUI.Button(new Rect(257, 460, 45, 25), "origin"))
                {
                    globalPosition = baseState.Translation;
                    globalQuaternion = baseState.Rotation;
                }

                // set current position and rotation of endeffector
                if (GUI.Button(new Rect(257, 490, 45, 25), "cPos"))
                {
                    globalPosition = SortIkServo[SortIkServo.Count - 1].ServoTransform.position;
                    globalQuaternion = SortIkServo[SortIkServo.Count - 1].ServoTransform.rotation;
                }
                //Debug.Log(string.Format("[TRF] {0} - Set/Reset position", 1010));


                // set position and rotation of endeffector ???

                Color[] color = { Color.red, Color.yellow };
                //if (data == null)
                //    Debug.Log(string.Format("[TRF] {0} - data == null", 1011));
                //else
                LineDiagram(diagramRectangle, data, color);
                //// [EXC 20:30:37.485]
                //// NullReferenceException: Object reference not set to an instance of an object
                //// IkRobotController.IkRobotController.LineDiagram(Rect rectangle, System.Collections.Generic.List`1 data, UnityEngine.Color[] color)
                //// IkRobotController.IkRobotController.OnWindow (Int32 windowID)
                //// UnityEngine.GUILayout+LayoutedWindow.DoWindow (Int32 windowID)
                //// UnityEngine.GUI.CallWindowDelegate (UnityEngine.WindowFunction func, Int32 id, Int32 instanceID, UnityEngine.GUISkin _skin, Int32 forceRect, Single width, Single height, UnityEngine.GUIStyle style)
                //// [ERR 20:30:37.485]
                //// GUI Error: You are pushing more GUIClips than you are popping.Make sure they are balanced)

                GUILayout.EndHorizontal();

                GUI.DragWindow();
            }
        }

        public void OnMsgWindow(int msgWindowID)
        {
            GUILayout.BeginHorizontal();

            GUI.Label(new Rect(30, msgWindowRectangle.height / 2 - 15, msgWindowRectangle.width - 60, 20), msgText);

            // Close window button
            if (GUI.Button(new Rect(180, 3, 17, 15), "x"))
            {
                msgWindowActive = false;
            }
            // set current position and rotation of endeffector
            if (GUI.Button(new Rect((200 - 45) / 2, 70, 45, 25), "OK"))
            {
                msgWindowActive = false;
            }

            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }

        public void MsgWindow(Vector2 pos, string title, string text)
        {
            msgWindowPosition = pos;
            msgWindowTitle = title;
            msgText = text;

            msgWindowActive = true;
        }

        public void AddInputValue(Rect rectangle, string valueInString, out string valueOutString, string name, out int valueInt, float labelLength)
        {
            GUI.Label(new Rect(rectangle.x, rectangle.y + Yoffset, labelLength, rectangle.height), name);
            valueOutString = GUI.TextField(new Rect(rectangle.x + labelLength, rectangle.y + Yoffset, rectangle.width, rectangle.height), valueInString);
            try
            {
                valueInt = int.Parse(valueOutString);
            }
            catch
            {
                valueOutString = valueInString;
                valueInt = int.Parse(valueOutString);
            }
            Yoffset += 20f;
        }

        public void AddInputValue(Rect rectangle, string valueInString, out string valueOutString, string name, out float valueFloat, float labelLength)
        {
            GUI.Label(new Rect(rectangle.x, rectangle.y + Yoffset, labelLength, rectangle.height), name);
            valueOutString = GUI.TextField(new Rect(rectangle.x + labelLength, rectangle.y + Yoffset, rectangle.width, rectangle.height), valueInString);
            try
            {
                valueFloat = float.Parse(valueOutString);
            }
            catch
            {
                valueOutString = valueInString;
                valueFloat = float.Parse(valueOutString);
            }
            Yoffset += 20f;
        }

        public void AddOutputValue(Rect rectangle, string name, float valueFloat, float labelLength)
        {
            GUI.Label(new Rect(rectangle.x, rectangle.y + Yoffset, labelLength, rectangle.height), name);
            GUI.TextField(new Rect(rectangle.x + labelLength, rectangle.y + Yoffset, rectangle.width, rectangle.height), valueFloat.ToString("0.000"));
            Yoffset += 20f;
        }

        public void AddOutputValue(Rect rectangle, string name, float valueFloat, float labelLength, float fieldLength)
        {
            GUI.Label(new Rect(rectangle.x, rectangle.y + Yoffset, labelLength, rectangle.height), name);
            GUI.TextField(new Rect(rectangle.x + labelLength, rectangle.y + Yoffset, fieldLength, rectangle.height), valueFloat.ToString("0.000"));
            Yoffset += 20f;
        }

        private void AddOutputValue(Rect rectangle, string name, Vector3 valueVector3, float labelLength, float fieldLength)
        {
            GUI.Label(new Rect(rectangle.x, rectangle.y + Yoffset, labelLength, rectangle.height), name);
            GUI.TextField(new Rect(rectangle.x + labelLength, rectangle.y + Yoffset, rectangle.width + fieldLength, rectangle.height), VectorToString(valueVector3, "0.00"));
            Yoffset += 20f;
        }

        private float FixValuesSlider(Rect rectangle, float nameLenght, string name, float value, float[] Values)
        {
            float mindiff = Values[Values.Length - 1] - Values[0];
            int index = 0;
            Rect valueRect = rectangle;
            valueRect.x += valueRect.width + nameLenght + 5;
            GUI.Label(rectangle, name);
            GUI.Label(valueRect, value.ToString());
            rectangle.x += nameLenght;
            rectangle.y += 7;
            value = GUI.HorizontalSlider(rectangle, value, Values[0], Values[Values.Length - 1]);
            for (int i = 0; i < Values.Length; i++)
            {
                if (Math.Abs(value - Values[i]) < mindiff)
                {
                    mindiff = Math.Abs(value - Values[i]);
                    index = i;
                }
            }
            value = Values[index];

            return value;
        }

        private float FixValuesLogSlider(Rect rectangle, float nameLenght, string name, float value, float[] Values)
        {
            float[] LogValues = new float[Values.Length];

            for (int i = 0; i < Values.Length; i++)
            {
                if (i == 0)
                    LogValues[0] = Mathf.Log10((Values[1] / 2f));
                else
                    LogValues[i] = Mathf.Log10(Values[i]);
            }

            float mindiff = Values[Values.Length - 1] - Values[0];
            int index = 0;
            Rect valueRect = rectangle;

            valueRect.x += valueRect.width + nameLenght + 5f;
            GUI.Label(rectangle, name);
            GUI.Label(valueRect, value.ToString());
            rectangle.x += nameLenght;

            Rect fixLine = new Rect(rectangle);
            fixLine.width = 10f;
            fixLine.y += 2f;
            Rect fixLineRect = new Rect(fixLine);

            for (int i = 0; i < LogValues.Length; i++)
            {
                float linePosition = (rectangle.width - 12f) / Mathf.Abs(LogValues[0] - LogValues[LogValues.Length - 1]) * (LogValues[i] + LogValues[0] * -1f) + 2f;
                fixLineRect.x = fixLine.x + linePosition;
                GUI.Label(fixLineRect, "│");
            }

            rectangle.y += 6f;
            float valueLog;
            if (value == 0f)
                valueLog = LogValues[0];
            else
                valueLog = Mathf.Log10(value);

            valueLog = GUI.HorizontalSlider(rectangle, valueLog, LogValues[0], LogValues[Values.Length - 1]);
            value = (float)Math.Pow(10d, (double)valueLog);
            for (int i = 0; i < Values.Length; i++)
            {
                if (Math.Abs(value - Values[i]) < mindiff)
                {
                    mindiff = Math.Abs(value - Values[i]);
                    index = i;
                }
            }
            if (index == 0)
                value = 0f;
            else
                value = Values[index];

            return value;
        }

        private void DrawRectangle(Rect rectangle, Color color)
        {
            Drawing.DrawLine(new Vector2(rectangle.x, rectangle.y), new Vector2(rectangle.x + rectangle.width, rectangle.y), color);
            Drawing.DrawLine(new Vector2(rectangle.x, rectangle.y), new Vector2(rectangle.x, rectangle.y + rectangle.height), color);
            Drawing.DrawLine(new Vector2(rectangle.x + rectangle.width, rectangle.y), new Vector2(rectangle.x + rectangle.width, rectangle.y + rectangle.height), color);
            Drawing.DrawLine(new Vector2(rectangle.x, rectangle.y + rectangle.height), new Vector2(rectangle.x + rectangle.width, rectangle.y + rectangle.height), color);
        }

        private void LineDiagram(Rect rectangle, List<float> data, Color color)
        {
            // set border deathzone
            Vector2 border = new Vector2(5, 5);
            Rect realDataField = new Rect(new Vector2(rectangle.x + border.x, rectangle.y + border.y), new Vector2(rectangle.width - (2 * border.x), rectangle.height - (2 * border.y)));
            Rect activeDataField = new Rect(new Vector2(0, 0), new Vector2(rectangle.width - (2 * border.x), rectangle.height - (2 * border.y)));
            // draw border line
            DrawRectangle(rectangle, Color.white);

            //DrawRectangle (realDataField, Color.yellow);

            GUI.BeginGroup(realDataField);
            //GUILayout.BeginArea(realDataField);

            // draw diagram data
            if (data.Count > 1)
            {
                for (int i = 1; i < data.Count; i++)
                {
                    Drawing.DrawLine(new Vector2((float)(i - 1), activeDataField.height - data[i - 1]), new Vector2((float)(i), activeDataField.height - data[i]), color);
                }
            }

            GUI.EndGroup();
            //GUILayout.EndArea();
        }

        private void LineDiagram(Rect rectangle, List<List<float>> data, Color[] color)
        {
            // set border deathzone
            Vector2 border = new Vector2(5, 5);
            Rect realDataField = new Rect(new Vector2(rectangle.x + border.x, rectangle.y + border.y), new Vector2(rectangle.width - (2 * border.x), rectangle.height - (2 * border.y)));
            Rect activeDataField = new Rect(new Vector2(0, 0), new Vector2(rectangle.width - (2 * border.x), rectangle.height - (2 * border.y)));
            // draw border line
            DrawRectangle(rectangle, Color.white);

            //DrawRectangle (realDataField, Color.yellow);

            GUI.BeginGroup(realDataField);
            //GUILayout.BeginArea(realDataField);

            if (data.Count > 0)
                for (int j = 0; j < data.Count; j++)
                {
                    // draw diagram data
                    if (data[j].Count > 1)
                    {
                        for (int i = 1; i < data[j].Count; i++)
                        {
                            Drawing.DrawLine(new Vector2((float)(i - 1), activeDataField.height - (data[j][i - 1] + 1)), new Vector2((float)(i), activeDataField.height - (data[j][i] + 1)), color[j]);
                        }
                    }
                }

            GUI.EndGroup();
            //GUILayout.EndArea();
        }
        #endregion GUI

        private bool ImplementServoRotation(List<IKservo> Servos, float[] theta, float speed = 0.25f)
        {
            bool success = true;
            float servoTheta = 0f;

            for (int j = 0; j < SortIkServo.Count - 1; j++)
                if (!double.IsNaN(theta[j]))
                {
                    // blocking fullrotate of servo
                    if (theta[j] > 180f)
                        servoTheta = theta[j] - 360f;
                    else if (theta[j] < -180f)
                        servoTheta = theta[j] + 360f;
                    else
                        servoTheta = theta[j];

                    //SortIkServo[j].iservo.Position
                    //SortIkServo[j].iservo.MoveTo(theta[j], speed);
                    SortIkServo[j].iservo.MoveTo(servoTheta, speed);
                }
                else
                    success = false;
            return success;
        }

        private void GetChildPartRecursive(Part obj)
        {
            if (null == obj)
                return;

            foreach (Part child in obj.children)
            {
                if (null == child)
                    continue;
                listOfChildrenPart.Add(child);
                GetChildPartRecursive(child);
            }
        }

        private string VectorToString(Vector3 vector, string format)
        {
            string szoveg = "";

            szoveg = string.Format("( " + vector.x.ToString(format) + ", " + vector.y.ToString(format) + ", " + vector.z.ToString(format) + " )");

            return szoveg;
        }

        private string QuaternionToString(Quaternion quaternion, string format)
        {
            string szoveg = "";

            szoveg = string.Format("( " + quaternion.w.ToString(format) + ", " + quaternion.x.ToString(format) + ", " + quaternion.y.ToString(format) + ", " + quaternion.z.ToString(format) + " )");

            return szoveg;
        }

        #region Kinematics
        public bool InverseKinematics(Vector3 targetPosition, Quaternion targetOrientation, float[] angles)
        {
            // calculate different of distance and -rotation
            LocationRotationDiff locRotDiff = DistanceAndAngleFromTarget(targetPosition, targetOrientation, angles);

            Distance = locRotDiff.LocationDiff;
            Angle = locRotDiff.RotationDiff;

            // if different lower than threshold (distance and rotation) then stop
            if (locRotDiff.LocationDiff < DistanceThreshold && locRotDiff.RotationDiff < AngleThreshold)
                return true;

            for (int i = 0; i < SortIkServo.Count; i++)
            {
                // Gradient descent
                // Update : Solution -= LearningRate * Gradient

                // calculate partial gradient
                LocationRotationDiff gradient = PartialGradient(targetPosition, targetOrientation, angles, i);

                // calculate dinamic learning rate of location different 
                DinLearningRatePos = locRotDiff.LocationDiff / MaxPosErr * 100.0f;
                // calculate dinamic learning rate of rotation different
                DinLearningRateOri = locRotDiff.RotationDiff / MaxOriErr * 100.0f;

                // calculate angle of joint with location gradient and rotation gradient - why sum ???
                angles[i] -= DinLearningRatePos * gradient.LocationDiff + DinLearningRateOri * gradient.RotationDiff;

                // Clamp - limited angle of joint
                angles[i] = Mathf.Clamp(angles[i], SortIkServo[i].MinAngle, SortIkServo[i].MaxAngle);

                // if different lower than threshold (distance and rotation) then stop
                if (locRotDiff.LocationDiff < DistanceThreshold && locRotDiff.RotationDiff < AngleThreshold)
                    return true;
            }
            return false;
        }

        public LocationRotationDiff PartialGradient(Vector3 targetPosition, Quaternion targetOrientation, float[] angles, int i)
        {
            // Saves the angle,
            // it will be restored later
            float angle = angles[i];
            LocationRotationDiff gradient = new LocationRotationDiff();

            // calculate different
            LocationRotationDiff f_xGlobal = DistanceAndAngleFromTarget(targetPosition, targetOrientation, angles);
            // Gradient : [F(x+SamplingDistance) - F(x)] / h

            // change angles with samplingangle (a little bit)
            angles[i] += SamplingAngle;

            // recalculate different
            LocationRotationDiff f_x_plus_dGlobal = DistanceAndAngleFromTarget(targetPosition, targetOrientation, angles);

            // calculate gradient of position
            gradient.LocationDiff = (f_x_plus_dGlobal.LocationDiff - f_xGlobal.LocationDiff) / SamplingAngle;
            // calculate gradient of orientation
            gradient.RotationDiff = (f_x_plus_dGlobal.RotationDiff - f_xGlobal.RotationDiff) / SamplingAngle;

            // Restore angles
            angles[i] = angle;

            return gradient;
        }

        public LocationRotationDiff DistanceAndAngleFromTarget(Vector3 targetPosition, Quaternion targetOrientation, float[] angles)
        {
            LocationRotationDiff different = new LocationRotationDiff();

            VectorSM DistanceAndAngle = ForwardKinematics(angles, SortIkServo);

            different.LocationDiff = Vector3.Distance(DistanceAndAngle.Translation, targetPosition);

            different.RotationDiff = Quaternion.Angle(DistanceAndAngle.Rotation, targetOrientation);

            return different;
        }

        public VectorSM ForwardKinematics(float[] angles, List<IKservo> Servos)
        {
            VectorSM endEffector = new VectorSM();
            // Vector3 prevPoint = Servos[0].Position;
            Vector3 prevPoint = Servos[0].ServoTransform.position;
            //Vector3 prevPoint = part.transform.position;

            // Quaternion rotation = Quaternion.identity;
            // Quaternion rotation = Servos[0].ServoTransform.rotation * Quaternion.Inverse(Servos[0].localRotation);
            // Quaternion rotation = Servos[0].ServoTransform.rotation * Quaternion.Inverse(Servos[0].Rotation);
            Quaternion rotation = part.transform.rotation * Quaternion.Inverse(partRotation);

            for (int i = 1; i < Servos.Count; i++)
            {
                // change rotation by previous servo angle - 'relative change'
                rotation *= Quaternion.AngleAxis(angles[i - 1], Servos[i - 1].fkParams.Axis);
                // change position by ( servo's ParentOffset modified by previous servo angle ) - 'relative change'
                Vector3 nextPoint = prevPoint + rotation * Servos[i - 1].fkParams.Rotation * Servos[i].fkParams.ParentOffset;
                // set position of servotransform's sign
                servoGimbal[i - 1].transform.position = prevPoint;
                // set rotation of servotransform's sign
                servoGimbal[i - 1].transform.rotation = rotation * Servos[i - 1].fkParams.Rotation;

                prevPoint = nextPoint;
            }
            // set endeffector position data
            endEffector.Translation = prevPoint;
            // set endeffector rotation data
            endEffector.Rotation = rotation * Servos[Servos.Count - 2].fkParams.Rotation;
            return endEffector;
        }
        #endregion Kinematics
    }
}
