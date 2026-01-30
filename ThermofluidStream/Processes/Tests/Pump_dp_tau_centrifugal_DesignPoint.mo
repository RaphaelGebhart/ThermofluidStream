within ThermofluidStream.Processes.Tests;
model Pump_dp_tau_centrifugal_DesignPoint "Tests dp_tau_isentrop design point"
  extends Modelica.Icons.Example;

  replaceable package Medium = ThermofluidStream.Media.myMedia.Examples.TwoPhaseWater
    constrainedby ThermofluidStream.Media.myMedia.Interfaces.PartialMedium "Medium model"
    annotation (
      choicesAllMatching=true,
      Documentation(info="<html>
<p>
Medium model for the test. Should be incompressible or with low compressibility.
</p>
</html>"));
  parameter SI.Pressure p_in=100000 "Inlet pressure";
  parameter SI.Temperature T_in=293.15 "Inlet temperature";
  parameter SI.Density rho_in = Medium.density_pT(p_in, T_in) "Inlet density";
  parameter SI.Height TDH_D=3.6610 "Design pressure head (max efficiency)";
  parameter SI.VolumeFlowRate V_flow_D=3.06e-3 "Design volume flow (max efficiency)";
  parameter SI.AngularVelocity omega_D=314.2 "Design angular velocity";
  parameter SI.Pressure dp_D = TDH_D*rho_in*Modelica.Constants.g_n "Design pressure difference";
  final parameter SI.MassFlowRate m_flow_D = V_flow_D*rho_in "Design mass flow rate";

  ThermofluidStream.Boundaries.Source source(
    redeclare package Medium = Medium,
    T0_par=T_in,
    p0_par=p_in)
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
  ThermofluidStream.Boundaries.Sink sink(
    redeclare package Medium = Medium, pressureFromInput=true)
    annotation (Placement(transformation(extent={{0,50},{20,70}})));

  inner ThermofluidStream.DropOfCommons dropOfCommons(L=1,
    p_min=10000,
    displayInstanceNames=true,
    displayParameters=true)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={110,90})));

  ThermofluidStream.Processes.Pump pump(
    redeclare package Medium = Medium,
    L=100000,
    omega_from_input=true,
    initM_flow=ThermofluidStream.Utilities.Types.InitializationMethods.state,
    redeclare function dp_tau_pump = ThermofluidStream.Processes.Internal.TurboComponent.dp_tau_centrifugal (
        useLegacyReynolds=false,
        TDH_D=TDH_D,
        V_flow_D=V_flow_D,
        omega_D=omega_D)) annotation (Placement(transformation(extent={{-70,50},{-50,70}})));

  Modelica.Blocks.Sources.Constant rotationalSpeed(k=omega_D) annotation (Placement(transformation(extent={{-100,10},{-80,30}})));
  Modelica.Blocks.Sources.Constant
                               massFlowRate(k=m_flow_D) annotation (Placement(transformation(extent={{100,50},{80,70}})));
  Modelica.Blocks.Math.Feedback feedback annotation (Placement(transformation(extent={{70,50},{50,70}})));
  Modelica.Blocks.Continuous.PI PI(
    k=-1e8,
    T=0.05,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=p_in) annotation (Placement(transformation(extent={{40,50},{20,70}})));
  ThermofluidStream.Sensors.SingleFlowSensor flowSensor(
    redeclare package Medium = Medium,
    quantity=ThermofluidStream.Sensors.Internal.Types.MassFlowQuantities.m_flow_kgps,
    outputValue=true) annotation (Placement(transformation(extent={{-40,70},{-20,50}})));
  Boundaries.Source                   source1(
    redeclare package Medium = Medium,
    T0_par=T_in,
    p0_par=p_in)
    annotation (Placement(transformation(extent={{-100,-30},{-80,-10}})));
  Boundaries.Sink                   sink1(
    redeclare package Medium = Medium,
    pressureFromInput=false,
    p0_par=p_in + dp_D)
    annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
  ThermofluidStream.Processes.Pump pump1(
    redeclare package Medium = Medium,
    L=100000,
    omega_from_input=true,
    initM_flow=ThermofluidStream.Utilities.Types.InitializationMethods.state,
    redeclare function dp_tau_pump = Internal.TurboComponent.dp_tau_centrifugal (
        useLegacyReynolds=false,
        TDH_D=TDH_D,
        V_flow_D=V_flow_D,
        omega_D=omega_D)) annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
  Modelica.Blocks.Math.Feedback feedback1
                                         annotation (Placement(transformation(extent={{0,-50},{-20,-70}})));
  Modelica.Blocks.Continuous.PI PI1(
    k=1,
    T=0.1,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=320)
               annotation (Placement(transformation(extent={{-30,-70},{-50,-50}})));
  Sensors.SingleFlowSensor flowSensor1(
    redeclare package Medium = Medium,
    quantity=ThermofluidStream.Sensors.Internal.Types.MassFlowQuantities.m_flow_kgps,
    outputValue=true) annotation (Placement(transformation(extent={{-40,-10},{-20,-30}})));
  Modelica.Blocks.Sources.Constant
                               massFlowRate1(k=m_flow_D)
                                                        annotation (Placement(transformation(extent={{40,-70},{20,-50}})));
  FlowControl.CheckValve checkValve(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
equation
  connect(source.outlet, pump.inlet) annotation (Line(
      points={{-80,60},{-70,60}},
      color={28,108,200},
      thickness=0.5));
  connect(rotationalSpeed.y, pump.omega_input) annotation (Line(points={{-79,20},{-60,20},{-60,48}}, color={0,0,127}));
  connect(feedback.y, PI.u) annotation (Line(points={{51,60},{42,60}}, color={0,0,127}));
  connect(sink.p0_var, PI.y) annotation (Line(points={{12,60},{19,60}}, color={0,0,127}));
  connect(feedback.u1, massFlowRate.y) annotation (Line(points={{68,60},{79,60}}, color={0,0,127}));
  connect(pump.outlet, flowSensor.inlet) annotation (Line(
      points={{-50,60},{-40,60}},
      color={28,108,200},
      thickness=0.5));
  connect(sink.inlet, flowSensor.outlet) annotation (Line(
      points={{0,60},{-20,60}},
      color={28,108,200},
      thickness=0.5));
  connect(flowSensor.value_out, feedback.u2) annotation (Line(points={{-22,54},{-10,54},{-10,32},{60,32},{60,52}}, color={0,0,127}));
  connect(source1.outlet, pump1.inlet) annotation (Line(
      points={{-80,-20},{-70,-20}},
      color={28,108,200},
      thickness=0.5));
  connect(feedback1.y, PI1.u) annotation (Line(points={{-19,-60},{-28,-60}}, color={0,0,127}));
  connect(pump1.outlet, flowSensor1.inlet) annotation (Line(
      points={{-50,-20},{-40,-20}},
      color={28,108,200},
      thickness=0.5));
  connect(flowSensor1.value_out, feedback1.u2) annotation (Line(points={{-22,-26},{-10,-26},{-10,-52}}, color={0,0,127}));
  connect(PI1.y, pump1.omega_input) annotation (Line(points={{-51,-60},{-60,-60},{-60,-32}}, color={0,0,127}));
  connect(massFlowRate1.y, feedback1.u1) annotation (Line(points={{19,-60},{-2,-60}}, color={0,0,127}));
  connect(flowSensor1.outlet, checkValve.inlet) annotation (Line(
      points={{-20,-20},{0,-20}},
      color={28,108,200},
      thickness=0.5));
  connect(checkValve.outlet, sink1.inlet) annotation (Line(
      points={{20,-20},{40,-20}},
      color={28,108,200},
      thickness=0.5));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6, Interval=0.01),
        Documentation(info="<html>
  <p>
    Test model for the design point of the pump characteristics according to
    <a href=\"modelica://ThermofluidStream.Processes.Internal.TurboComponent.dp_tau_centrifugal\">
      dp_tau_centrifugal
    </a>:
  </p>
  
  <ul>
    <li>
      head at design flow and design rotational speed
    </li>
    <li>
      rotational speed at design flow and design head
    </li>
  </ul>

  <p>
    The model compares the legacy formulation
    (<code>useLegacyReynolds = true</code>) with the corrected formulation
    (<code>useLegacyReynolds = false</code>).
  </p>



</html>", revisions="<html>
  <ul>
    <li>
      2026, by Raphael Gebhart (raphael.gebhart@dlr.de):<br>
      Initial version.
    </li>
  </ul>
</html>"),
    Diagram(coordinateSystem(extent={{-120,-100},{120,100}})),
    __Dymola_Commands(file(autoRun=true) = "Resources/Scripts/ThermofluidStream.Processes.Tests.Pump_dp_tau_centrifugal_DesignPoint.mos"));
end Pump_dp_tau_centrifugal_DesignPoint;
