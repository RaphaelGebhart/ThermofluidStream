﻿within ThermofluidStream.Sensors;
model DifferenceSensorSelect32
  "v3.2 of DifferenceSensorSelect"
  import ThermofluidStream.Sensors.Internal.Types.Quantities;
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
  parameter SI.Density rho_min = dropOfCommons.rho_min "Minimum allowed density"
    annotation(Dialog(tab="Advanced", group="Regularization"));
  parameter Quantities quantity "Quantity the sensor measures";

final parameter String quantityString=
  if quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.T_K then "T in K"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.T_C then "T in °C"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.p_Pa then "p in Pa"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.p_bar then "p in bar"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.rho_kgpm3 then "d in kg/m3"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.v_m3pkg then "v in m3/kg"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.h_Jpkg then "h in J/kg"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.s_JpkgK then "s in J/(kg.K)"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.a_mps then "Velocity of sound in m/s"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.cv_JpkgK then "cv in J/(kg.K)"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.cp_JpkgK then "cp in J/(kg.K)"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.kappa_1 then "kappa"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.MM_kgpmol then  "M in kg/mol"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.r_Pa then "r in Pa"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.r_bar then "r in bar"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.p_total_Pa then "(p+r) in Pa"
  elseif quantity == ThermofluidStream.Sensors.Internal.Types.Quantities.p_total_bar then "(p+r) in bar"
  else "error";

  parameter Boolean outputValue = false "Enable sensor-value output"
    annotation(Dialog(group="Output Value"));
  parameter Boolean filter_output = false "Filter sensor-value to break algebraic loops"
    annotation(Dialog(group="Output Value", enable=outputValue));
  parameter InitMode init=InitMode.steadyState "Initialization mode for sensor lowpass"
    annotation(Dialog(tab="Initialization", enable=filter_output));
  parameter Real value_0(unit=Internal.getUnit(quantity)) = 0 "Initial output state of sensor"
    annotation(Dialog(tab="Initialization", enable=filter_output and init==InitMode.state));
  parameter SI.Time TC = 0.1 "PT1 time constant"
    annotation(Dialog(tab="Advanced", enable=outputValue and filter_output));

  Interfaces.Inlet inletA(redeclare package Medium=MediumA)
    annotation (Placement(transformation(extent={{-20, -20},{20, 20}}, origin={-100,60}),
        iconTransformation(extent={{-120,40},{-80,80}})));
  Interfaces.Inlet inletB(redeclare package Medium=MediumB)
    annotation (Placement(transformation(extent={{-20, -20},{20, 20}}, origin={-100,0}),
        iconTransformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Interfaces.RealOutput value_out(unit=Internal.getUnit(quantity)) = value if outputValue "Difference of measured quantity [variable]"
    annotation (Placement(transformation(extent={{70,-10},{90,10}}), iconTransformation(extent={{70,-10},{90,10}})));

  output Real value(unit=Internal.getUnit(quantity)) "Computed difference in the selected quantity";

  Real valueA(unit=Internal.getUnit(quantity));
  Real valueB(unit=Internal.getUnit(quantity));

protected
  Real direct_value(unit=Internal.getUnit(quantity));

  function getQuantityA = Internal.getQuantity(redeclare package Medium=MediumA) "Quantity compute function A"
    annotation (Documentation(info="<html>
      <p>This function computes the selected quantity from state. r and rho_min are neddet for the quantities r/p_total and v respectively.</p>
      </html>"));
  function getQuantityB = Internal.getQuantity(redeclare package Medium=MediumB) "Quantity compute function B"
    annotation (Documentation(info="<html>
      <p>This function computes the selected quantity from state. r and rho_min are neddet for the quantities r/p_total and v respectively.</p>
      </html>"));

initial equation
  if filter_output and init==InitMode.steadyState then
    value= direct_value;
  elseif filter_output then
    value = value_0;
  end if;

equation

  inletA.m_flow = 0;
  inletB.m_flow = 0;

  valueA =  getQuantityA(inletA.state, inletA.r, quantity, rho_min);
  valueB =  getQuantityB(inletB.state, inletB.r, quantity, rho_min);

  direct_value = valueA - valueB;

  if filter_output then
    der(value) * TC = direct_value-value;
  else
    value = direct_value;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={
        Text(visible=displayInstanceName,
          extent={{-150,120},{150,80}},
          textString="%name",
          textColor={0,0,255}),
        Rectangle(
          extent={{-54,24},{66,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-80,0},{0,0}},
          color={28,108,200},
          thickness=0.5),
        Rectangle(
          extent={{-60,30},{60,-30}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,30},{60,-30}},
          textColor={28,108,200},
          textString=DynamicSelect("value", String(
              value,
              format="1."+String(digits)+"f"))),
        Text(
          extent={{-150,-70},{150,-40}},
          textColor={0,0,0},
          textString=quantityString),
        Line(
          points={{-80,60},{-80,0}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-100,60},{-80,60}},
          color={28,108,200},
          thickness=0.5),
        Ellipse(
          extent={{-120,-20},{-100,-40}},
          lineColor={28,108,200},
          lineThickness=0.5),
        Line(visible=outputValue,
          points={{60,0},{78,0}},
          color={0,0,127}),
        Line(
          points={{-118,-30},{-102,-30}},
          color={28,108,200},
          thickness=0.5),
        Ellipse(
          extent={{-120,40},{-100,20}},
          lineColor={28,108,200},
          lineThickness=0.5),
        Line(
          points={{-118,30},{-102,30}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-8,0},{8,0}},
          color={28,108,200},
          thickness=0.5,
          origin={-110,30},
          rotation=90)}),
    Diagram(coordinateSystem(preserveAspectRatio=true)),
    Documentation(info="<html>
<p>Sensor for measuring the difference of a selectable quantity between two fluid streams.</p>
<p>This sensor can be connected to two fluid streams without a junction.</p>
</html>"));
end DifferenceSensorSelect32;