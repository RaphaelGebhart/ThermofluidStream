within ThermofluidStream.Processes.Tests;
model CentrifugalPump "Basic cooling cycle with a load"
extends Modelica.Icons.Example;

  replaceable package Medium_liquid = ThermofluidStream.Media.myMedia.Water.ConstantPropertyLiquidWater constrainedby
    ThermofluidStream.Media.myMedia.Interfaces.PartialMedium
    "Liquid model"
    annotation(choicesAllMatching = true);

  replaceable package Medium_air = ThermofluidStream.Media.myMedia.Air.DryAirNasa
    constrainedby ThermofluidStream.Media.myMedia.Interfaces.PartialMedium
    "Air model"
    annotation(choicesAllMatching = true);

  ThermofluidStream.HeatExchangers.CounterFlowNTU heatExchanger(
    redeclare package MediumA = Medium_air,
    redeclare package MediumB = Medium_liquid,
    A=10) annotation (Placement(transformation(extent={{70,-46},{90,-26}})));
  ThermofluidStream.Boundaries.VolumeFlex flexVolume(
    redeclare package Medium = Medium_liquid,
    V_ref=0.02,
    p_start=100000,
    T_start=278.15) annotation (Placement(transformation(extent={{30,-40},{10,-20}})));
  ThermofluidStream.Processes.ThermalConvectionPipe thermalConvectionPipe(
    redeclare package Medium = Medium_liquid,
    r=0.005,
    l=1) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-90,0})));
  ThermofluidStream.Processes.FlowResistance flowResistance(
    redeclare package Medium = Medium_liquid,
    initM_flow=ThermofluidStream.Utilities.Types.InitializationMethods.state,
    r=0.02,
    l=100,
    redeclare function pLoss = ThermofluidStream.Processes.Internal.FlowResistance.laminarTurbulentPressureLoss)
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm1(
    displayInstanceName=false,
    redeclare package Medium = Medium_liquid,
    temperatureUnit="degC")
    annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm2(
    displayInstanceName=false,
    redeclare package Medium = Medium_liquid,
    digits=3,
    temperatureUnit="degC")
    annotation (Placement(transformation(extent={{0,-30},{-20,-10}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm3(
    displayInstanceName=false,
    redeclare package Medium = Medium_liquid,
    digits=3,
    outputMassFlowRate=false,
    temperatureUnit="degC")
    annotation (Placement(transformation(extent={{-58,-30},{-78,-10}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm4(
    displayInstanceName=false,
    redeclare package Medium = Medium_liquid,
    temperatureUnit="degC")
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-20,40})));
  ThermofluidStream.Boundaries.Source source(
    redeclare package Medium = Medium_air,
    T0_par=283.15,
    p0_par=200000)
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={8,-60})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm5(
    displayInstanceName=false,
    redeclare package Medium = Medium_air,
    temperatureUnit="degC")
    annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={52,-52})));
  ThermofluidStream.Boundaries.Sink sink(
    redeclare package Medium = Medium_air, p0_par=100000)
                   annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={166,-42})));

  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=500, T(start=373.15, fixed=true))
    annotation (Placement(transformation(extent={{-130,0},{-110,20}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
    annotation (Placement(transformation(extent={{-160,-10},{-140,10}})));
  inner ThermofluidStream.DropOfCommons dropOfCommons(displayInstanceNames=true,
                                                      displayParameters=true)
    annotation (Placement(transformation(extent={{-200,180},{-180,200}})));
  ThermofluidStream.Processes.CentrifugalPump centrifugalPump(
    redeclare package Medium = Medium_liquid,
    dataFromMeasurements=true,
    redeclare
      ThermofluidStream.Processes.Internal.CentrifugalPump.CoefficientsData.Wilo.CronolineIL80slash220dash4slash4
      coefficients,
    redeclare
      ThermofluidStream.Processes.Internal.CentrifugalPump.MeasurementData.Wilo.CronolineIL80slash220dash4slash4
      measurements,
    pumpMode=ThermofluidStream.Processes.Internal.Types.PumpMode.flange,
    setpointFromInput=true,
    phi(fixed=true)) annotation (Placement(transformation(extent={{-30,-40},{-50,-20}})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm6(
    displayInstanceName=false,
    redeclare package Medium = Medium_liquid, temperatureUnit="degC")
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=180,
        origin={110,-20})));
  ThermofluidStream.Sensors.MultiSensor_Tpm multiSensor_Tpm7(
    displayInstanceName=false,
    redeclare package Medium = Medium_air,
    temperatureUnit="degC")
    annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={110,-52})));
  ThermofluidStream.Utilities.showRealValue showRealValue1(
    use_numberPort=false,
    description="eff",
    number=heatExchanger.effectiveness) annotation (Placement(transformation(extent={{70,-20},{90,0}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
    table=[0,5000; 2000,5000;
        2000.01,10000; 4000,10000; 4000.01,25000; 6000,25000; 6000.01,5000; 10000,
        5000])
    annotation (Placement(transformation(extent={{-190,-10},{-170,10}})));
  ThermofluidStream.Utilities.Icons.DLRLogo dLRLogo annotation (Placement(transformation(extent={{164,126},{200,162}})));
  ThermofluidStream.Processes.FlowResistance flowResistance1(
    redeclare package Medium = Medium_air,
    initM_flow=ThermofluidStream.Utilities.Types.InitializationMethods.state,
    r=0.05,
    l=1,
    redeclare function pLoss = ThermofluidStream.Processes.Internal.FlowResistance.linearQuadraticPressureLoss (k=1e5))
    annotation (Placement(transformation(extent={{130,-52},{150,-32}})));
  Modelica.Blocks.Sources.Sine w_sine(
    amplitude=100,
    f=5e-4,
    offset=200,
    startTime=1e3) annotation (Placement(transformation(extent={{-140,-158},{-120,-138}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation (Placement(transformation(extent={{-130,-188},{-110,
            -168}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=false)
                                                    annotation (Placement(transformation(extent={{-108,-158},{-88,-138}})));
  Modelica.Blocks.Sources.Sine tau_sine(
    amplitude=0.5,
    f=5e-4,
    offset=1,
    startTime=1e3) annotation (Placement(transformation(extent={{-178,-188},{-158,-168}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=1,
    phi(fixed=true, start=0),
    w(fixed=true, start=0))                                                             annotation (Placement(transformation(extent={{-100,
            -188},{-80,-168}})));
  Modelica.Blocks.Sources.Sine m_flow_sine(
    amplitude=0.9,
    f=5e-4,
    offset=1,
    startTime=1e3) annotation (Placement(transformation(extent={{-120,-60},{-100,-40}})));
  Modelica.Blocks.Sources.Sine V_flow_sine(
    amplitude=0.9e-3,
    f=5e-4,
    offset=1e-3,
    startTime=1e3) annotation (Placement(transformation(extent={{-140,-80},{-120,-60}})));
  Modelica.Blocks.Sources.Sine dp_sine(
    amplitude=4e4,
    f=5e-4,
    offset=5e4,
    startTime=1e3) annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  Modelica.Blocks.Sources.Sine p_out_sine(
    amplitude=0.4e5,
    f=5e-4,
    offset=1.5e5,
    startTime=1e3) annotation (Placement(transformation(extent={{-120,-120},{-100,-100}})));
  Modelica.Blocks.Sources.Sine w_sine1(
    amplitude=100,
    f=5e-4,
    offset=200,
    startTime=1e3) annotation (Placement(transformation(extent={{-80,-140},{-60,-120}})));
equation
  connect(multiSensor_Tpm1.inlet, heatExchanger.outletB)
    annotation (Line(
      points={{60,-30},{70,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm1.outlet, flexVolume.inlet) annotation (Line(
      points={{40,-30},{30,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(flexVolume.outlet, multiSensor_Tpm2.inlet) annotation (Line(
      points={{10,-30},{0,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm3.outlet, thermalConvectionPipe.inlet) annotation (
      Line(
      points={{-78,-30},{-90,-30},{-90,-10}},
      color={28,108,200},
      thickness=0.5));
  connect(thermalConvectionPipe.outlet, multiSensor_Tpm4.inlet) annotation (
      Line(
      points={{-90,10},{-90,30},{-30,30}},
      color={28,108,200},
      thickness=0.5));
  connect(flowResistance.inlet, multiSensor_Tpm4.outlet) annotation (Line(
      points={{70,30},{-10,30}},
      color={28,108,200},
      thickness=0.5));
  connect(source.outlet, multiSensor_Tpm5.inlet) annotation (Line(
      points={{18,-60},{30,-60},{30,-42},{42,-42}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm5.outlet, heatExchanger.inletA)
    annotation (Line(
      points={{62,-42},{70,-42}},
      color={28,108,200},
      thickness=0.5));
  connect(thermalConvectionPipe.heatPort, heatCapacitor.port)
    annotation (Line(points={{-100,0},{-120,0}},  color={191,0,0}));
  connect(heatCapacitor.port, prescribedHeatFlow.port)
    annotation (Line(points={{-120,0},{-140,0}},   color={191,0,0}));
  connect(heatExchanger.inletB, multiSensor_Tpm6.outlet)
    annotation (Line(
      points={{90,-30},{100,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm6.inlet, flowResistance.outlet) annotation (Line(
      points={{120,-30},{130,-30},{130,30},{90,30}},
      color={28,108,200},
      thickness=0.5));
  connect(heatExchanger.outletA, multiSensor_Tpm7.inlet)
    annotation (Line(
      points={{90,-42},{100,-42}},
      color={28,108,200},
      thickness=0.5));
  connect(prescribedHeatFlow.Q_flow, combiTimeTable.y[1])
    annotation (Line(points={{-160,0},{-169,0}},   color={0,0,127}));

  connect(multiSensor_Tpm7.outlet, flowResistance1.inlet) annotation (Line(
      points={{120,-42},{130,-42}},
      color={28,108,200},
      thickness=0.5));
  connect(sink.inlet, flowResistance1.outlet) annotation (Line(
      points={{156,-42},{150,-42}},
      color={28,108,200},
      thickness=0.5));
  connect(multiSensor_Tpm3.inlet, centrifugalPump.outlet)
    annotation (Line(
      points={{-58,-30},{-50,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(centrifugalPump.inlet, multiSensor_Tpm2.outlet)
    annotation (Line(
      points={{-30,-30},{-20,-30}},
      color={28,108,200},
      thickness=0.5));
  connect(w_sine.y,speed. w_ref) annotation (Line(points={{-119,-148},{-110,-148}},
                                                                                color={0,0,127}));
  connect(tau_sine.y,torque. tau) annotation (Line(points={{-157,-178},{-132,-178}},
                                                                                   color={0,0,127}));
  connect(torque.flange,inertia. flange_a) annotation (Line(points={{-110,-178},{-100,-178}},
                                                                                          color={0,0,0}));
  connect(inertia.flange_b, centrifugalPump.flange)
    annotation (Line(points={{-80,-178},{-40,-178},{-40,-40}}, color={0,0,0}));
  annotation (
    experiment(
      StopTime=10000,
      Tolerance=1e-6,
      Interval=10,
      __Dymola_Algorithm="Dassl"), Diagram(coordinateSystem(extent={{-200,-200},{200,200}})),
    Documentation(info="<html>
<p>
You may connect any of the input signal to the <code>centrifugalPump</code> for the corresponding <code>pumpMode</code> (only initialization might has to be changed:
</p>
<ul>
<li><code>pumpMode = pumpMode.flange</code> enables a mechanical connector. Set <code>flowResistance.initM_flow = state, flowResistance.m_flow_0 = 0</code> and eventually 
<code>pump.phi(start=0, fixed=false)</code> to avoid warnings or errors.</li>
<li><code>pumpMode = pumpMode.flowControlled</code> enables to set mass or volume flow rate by parameter or from input signal. Set <code>flowResistance.initM_flow = none</code> to avoid warnings or errors.</li>
<li><code>pumpMode = pumpMode.pressureControlled</code> enables to set outlet pressure, pressure ratio or pressure difference by parameter or from input signal. Set <code>flowResistance.initM_flow = state, flowResistance.m_flow_0 = 0</code> 
to avoid warnings or errors.</li>
<li><code>pumpMode = pumpMode.speedControlled</code> enables to set angular velocity by parameter or from input signal. Set <code>flowResistance.initM_flow = state, flowResistance.m_flow_0 = 0</code> 
to avoid warnings or errors.</li>
</ul>
<p>
The pump mode is displayed on icon level by <code>m</code> for mass flow rate, <code>V</code> for volume flow rate, <code>dp</code> for pressure difference, <code>pr</code> for pressure ratio, 
<code>p</code> for outlet pressure and <code>w</code> for angular velocity.
</p>
<p>
<code>setpointFromInput</code> enables switching between setpoint by parameter or from input signal.
</p>
<p>
The centrifugal pump can be parameterized either with <code>measurements</code> (data) or with <code>coefficients</code>. 
There are already some examplary pumps for both records <code>measurements</code> and <code>coefficients</code>. User specific pumps can be added by creating a new <code>measurements</code> record,
whereas <code>coefficients</code> records enable the user to generate a reasonable pump curve by scaling, e.g. by applying similarity laws.
</p>
<p>Owner: <a href=\"mailto:raphael.gebhart@dlr.de\">Raphael Gebhart</a></p>
</html>"));
end CentrifugalPump;