﻿within ThermofluidStream.Sensors;
model DifferenceSensor_Tp3 "v3 of DifferenceSensor_Tp"
  import InitMode = ThermofluidStream.Sensors.Internal.Types.InitializationModelSensor;

  extends ThermofluidStream.Utilities.DropOfCommonsPlus;

  replaceable package MediumA = Media.myMedia.Interfaces.PartialMedium
    "Medium model A"
    annotation (choicesAllMatching=true,
      Documentation(info="<html>
        <p>Medium Model for the positive input of the sensor. Make sure it is the same for the stream the sensors inputs are connected.</p>
        </html>"));
  replaceable package MediumB = Media.myMedia.Interfaces.PartialMedium
    "Medium model B"
    annotation (choicesAllMatching=true,
    Documentation(info="<html>
    <p>Medium Model for the negative input of the sensor. Make sure it is the same for the stream the sensors inputs are connected.</p>
      </html>"));

  parameter Integer digits(min=0) = 1 "Number of displayed digits";
  parameter ThermofluidStream.Sensors.Internal.Types.TemperatureUnit temperatureUnit = "K" "Unit for temperature measurement and output"
    annotation(choicesAllMatching = true, Evaluate = true);
  parameter ThermofluidStream.Sensors.Internal.Types.PressureUnit pressureUnit = "Pa" "Unit for pressure measurement and output"
    annotation(choicesAllMatching = true, Evaluate = true);

    final parameter String quantityString=
  if temperatureUnit == "K" then "T in K, p in "+pressureUnit
  elseif temperatureUnit == "degC"  then "T in °C, p in "+pressureUnit
  else "error";

  parameter Boolean outputTemperature = false "Enable temperature output"
    annotation(Dialog(group="Output Value"));
  parameter Boolean outputPressure = false "Enable pressure output"
    annotation(Dialog(group="Output Value"));
  parameter Boolean filter_output = false "Filter sensor-value to break algebraic loops"
    annotation(Dialog(group="Output Value", enable=(outputTemperature or outputPressure)));
  parameter InitMode init=InitMode.steadyState "Initialization mode for sensor lowpass"
    annotation(choicesAllMatching=true, Dialog(tab="Initialization", enable=filter_output));
  parameter Real p_0(final quantity="Pressure", final unit=pressureUnit) = 0 "Initial output pressure of sensor"
    annotation(Dialog(tab="Initialization", enable=filter_output and init==InitMode.state));
  parameter Real T_0(final quantity="ThermodynamicTemperature", final unit=temperatureUnit) = 0 "Initial output temperature of sensor"
    annotation(Dialog(tab="Initialization", enable=filter_output and init==InitMode.state));
  parameter SI.Time TC = 0.1 "PT1 time constant"
    annotation(Dialog(tab="Advanced", enable=(outputTemperature or outputPressure) and filter_output));

  Interfaces.Inlet inletA(redeclare package Medium=MediumA)
    annotation (Placement(transformation(extent={{-20, -20},{20, 20}}, origin={-100,80}), iconTransformation(extent={{-120,40},{-80,80}})));
  Interfaces.Inlet inletB(redeclare package Medium=MediumB)
    annotation (Placement(transformation(extent={{-20, -20},{20, 20}}, origin={-100,-80}), iconTransformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Interfaces.RealOutput T_out(final quantity="ThermodynamicTemperature", final unit=temperatureUnit) = T if outputTemperature "Difference of measured Temperature [variable]"
    annotation (Placement(transformation(extent={{70,20},{90,40}}), iconTransformation(extent={{70,20},{90,40}})));
  Modelica.Blocks.Interfaces.RealOutput p_out(final quantity="Pressure", final unit=pressureUnit) = p if outputPressure "Difference of measured pressure [variable]"
    annotation (Placement(transformation(extent={{70,-40},{90,-20}}), iconTransformation(extent={{70,-40},{90,-20}})));

  output Real p(final quantity="Pressure", final unit=pressureUnit);
  output Real T(final quantity="ThermodynamicTemperature", final unit=temperatureUnit);

protected
  Real direct_p; //unit intentional not given to avoid warning
  Real direct_T; //unit intentional not given to avoid warning

  Real pA; //unit intentional not given to avoid warning
  Real TA; //unit intentional not given to avoid warning
  Real pB; //unit intentional not given to avoid warning
  Real TB; //unit intentional not given to avoid warning

initial equation
  if filter_output and init==InitMode.steadyState then
    p=direct_p;
    T=direct_T;
  elseif filter_output then
    p=p_0;
    T=T_0;
  end if;

equation
  inletA.m_flow = 0;
  inletB.m_flow = 0;

  if temperatureUnit == "K" then
    TA =  MediumA.temperature(inletA.state);
    TB =  MediumB.temperature(inletB.state);
  elseif temperatureUnit == "degC" then
    TA =Modelica.Units.Conversions.to_degC(MediumA.temperature(inletA.state));
    TB =Modelica.Units.Conversions.to_degC(MediumB.temperature(inletB.state));
  end if;

  if pressureUnit == "Pa" then
    pA = MediumA.pressure(inletA.state);
    pB = MediumB.pressure(inletB.state);
  elseif pressureUnit == "bar" then
    pA =Modelica.Units.Conversions.to_bar(MediumA.pressure(inletA.state));
    pB =Modelica.Units.Conversions.to_bar(MediumB.pressure(inletB.state));
  end if;

  direct_T = TA - TB;
  direct_p = pA - pB;

  if filter_output then
    der(p) * TC = direct_p-p;
    der(T) * TC = direct_T-T;
  else
    p = direct_p;
    T = direct_T;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={
        Text(visible=displayInstanceName,
          extent={{-150,140},{150,100}},
          textString="%name",
          textColor={0,0,255}),
        Rectangle(
          extent={{-54,54},{66,-66}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-100,0},{0,0}},
          color={28,108,200},
          thickness=0.5),
        Rectangle(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,55},{60,5}},
          textColor={28,108,200},
          textString=DynamicSelect("T", String(
              T,
              format="1."+String(digits)+"f"))),
        Text(
          extent={{-60,-5},{60,-55}},
          textColor={28,108,200},
          textString=DynamicSelect("p", String(
              p,
              format="1."+String(digits)+"f"))),
        Text(
          extent={{-150,-110},{150,-80}},
          textColor={0,0,0},
          textString=quantityString),
        Line(
          points={{-100,60},{-80,60}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-108,-30},{-92,-30}},
          color={28,108,200},
          thickness=0.5),
        Ellipse(
          extent={{-110,100},{-90,80}},
          lineColor={28,108,200},
          lineThickness=0.5),
        Line(visible=outputTemperature,
          points={{60,30},{78,30}},
          color={0,0,127}),
        Line(visible=outputPressure,
          points={{60,-30},{78,-30}},
          color={0,0,127}),
        Line(
          points={{-108,90},{-92,90}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-8,0},{8,0}},
          color={28,108,200},
          thickness=0.5,
          origin={-100,90},
          rotation=90),
        Ellipse(
          extent={{-110,-20},{-90,-40}},
          lineColor={28,108,200},
          lineThickness=0.5),
        Line(
          points={{-10,-1.68234e-16},{50,0}},
          color={28,108,200},
          thickness=0.5,
          origin={-80,10},
          rotation=90)}),
    Diagram(coordinateSystem(preserveAspectRatio=true)),
    Documentation(info="<html>
<p>Sensor for measuring difference in temperature and pressure at once.</p>
<p>This sensor can be connected to two fluid streams without a junction.</p>
</html>"));
end DifferenceSensor_Tp3;