within ThermofluidStream.Boundaries.Internal;
partial model PartialTank "Partial Tank model for media that are partial gas and incompressible liquid"

  extends ThermofluidStream.Utilities.DropOfCommonsPlus;

  // ------------------ Parameters equivalent to PartialVolume ---------------------------------------------------------
  replaceable package Medium = Media.myMedia.Interfaces.PartialMedium "Medium model"
   annotation (
      choicesAllMatching=true, Documentation(info="<html>
<p>
Medium package used in the Volume. Make sure it is the same as the
inlets and outlets the volume is connected to.
</p>
</html>"));
  parameter Boolean useHeatport = false "=true, if heatport is enabled"
    annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Boolean useInlet = true "= true, if inlet is enabled"
    annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Boolean useOutlet = true "= true, if outlet is enabled"
    annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter SI.Area A = 1 "Heat transfer area"
    annotation(Dialog(enable=useHeatport));
  parameter SI.CoefficientOfHeatTransfer U = 200 "Thermal transmittance"
    annotation(Dialog(enable=useHeatport));

  parameter Boolean initialize_pressure = true "=true, if pressure is initialized"
    annotation(Dialog(tab= "Initialization",group="Pressure"),Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter SI.Pressure p_start = Medium.p_default "Initial pressure set value"
    annotation(Dialog(tab= "Initialization",group="Pressure", enable=initialize_pressure));
  parameter Boolean initialize_energy = true "= true, if internal energy is initialized"
    annotation(Dialog(tab= "Initialization",group="Temperature"),Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter SI.Temperature T_start = Medium.T_default "Initial Temperature set value"
    annotation(Dialog(tab= "Initialization",group="Temperature", enable=initialize_energy and (not use_hstart)));
  parameter Boolean initialize_Xi = true "=true, if mass fractions are iinitialized"
    annotation(Dialog(tab= "Initialization",group="Mass fractions"),Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Medium.MassFraction Xi_0[Medium.nXi] = Medium.X_default[1:Medium.nXi] "Initial mass fractions set values"
    annotation(Dialog(tab= "Initialization",group="Mass fractions", enable=initialize_Xi));
  parameter Boolean use_hstart = false "=true, if internal energy is initialized with specific enthalpy"
    annotation(Dialog(tab= "Initialization",group="Specific enthalpy", enable=initialize_energy),Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter SI.SpecificEnthalpy h_start = Medium.h_default "Initial specific enthalpy set value"
    annotation(Dialog(tab= "Initialization",group="Specific enthalpy", enable=initialize_energy and use_hstart));
  parameter Utilities.Units.Inertance L = dropOfCommons.L "Inertance of inlet/outlet"
    annotation (Dialog(tab="Advanced"));
  parameter Real k_volume_damping(unit="1", min=0) = dropOfCommons.k_volume_damping "Damping factor multiplicator"
    annotation(Dialog(tab="Advanced", group="Damping"));
  parameter SI.MassFlowRate m_flow_assert(max=0) = -dropOfCommons.m_flow_reg "Assertion threshold for negative massflow"
    annotation(Dialog(tab="Advanced"));
  parameter Boolean usePreferredMediumStates=false "=true, if preferred medium states are used"
    annotation(Dialog(tab="Advanced"),Evaluate=true, HideResult=true, choices(checkBox=true));

   // ---------------------------------------- New parameters ----------------------------------------------------------
  parameter Boolean initialize_LiquidMass = true "= true, if mass of the liquid medium component is initialized (only possible if initialize_Xi=false)"
    annotation(Dialog(tab= "Initialization",group="Mass fractions", enable=not initialize_Xi));
  parameter SI.Mass M_liq_start=0 "Initial mass of the liquid"
    annotation(Dialog(tab= "Initialization",group="Mass fractions", enable=initialize_LiquidMass));
  parameter SI.Length outletTransition=0.01 "Bandwidth for smooth transition between gas and liquid at outlet"
    annotation(Dialog(tab="Advanced"));
   parameter SI.Length tankCenter[3]={0,0,0} "Position of the tank center"
     annotation(Dialog(tab="General",group="Geometry"));
   parameter Integer N_inlets = 2 "Number of inlets"
     annotation(Dialog(tab="General",group="Geometry"));
   parameter Integer M_outlets = 2 "Number of outlets"
     annotation(Dialog(tab="General",group="Geometry"));
   parameter SI.Length inletPositions[N_inlets,3]={{0,0,0},{0,0,0}} "Positions of all inlets"
     annotation(Dialog(tab="General",group="Geometry"));
   parameter SI.Length outletPositions[M_outlets,3]={{0,0,0},{0,0,0}} "Positions of all outlets"
     annotation(Dialog(tab="General",group="Geometry"));
  parameter SI.Pressure p_ref = 1e5 "Reference pressure of tank when volume is measured";
  //The tank gets a bulk modulus to handle the stiffness of incompressible media better
  SI.Volume V_ref(displayUnit="l") "Volume of the tank at p_ref";
  parameter SI.BulkModulus K = 5e7 "Bulk modulus of tank (used also for stiffness modulation)";


  Interfaces.Inlet inlet[N_inlets](redeclare package Medium=Medium)
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
  Interfaces.Outlet outlet[M_outlets](redeclare package Medium=Medium)
    annotation (Placement(transformation(extent={{80,-20},{120,20}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(Q_flow=Q_flow, T=T_heatPort) if useHeatport
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));

  Medium.BaseProperties medium(preferredMediumStates=usePreferredMediumStates);

  SI.Volume V "Volume";
  SI.Volume V_liquid "Volume of liquid";

  //SI.Volume V_gas "Gas volume in Tank";

  //setting the state is to prohibit dynamic state selection e.g. in VolumesDirectCoupling
  SI.Mass M(stateSelect=if usePreferredMediumStates then StateSelect.default else StateSelect.always) = V*medium.d "Mass";
  SI.Mass MXi[Medium.nXi](each stateSelect=if usePreferredMediumStates then StateSelect.default else StateSelect.always) = M*medium.Xi "Masses of species";
  SI.Energy U_med(stateSelect=if usePreferredMediumStates then StateSelect.default else StateSelect.always) = M*medium.u "Internal energy";

  SI.HeatFlowRate Q_flow "Heat flow rate";
  SI.Power W_v "Work due to change of volume";

  SI.Length centreOfMass[3];

  SI.Length staticHeadInlets[N_inlets] "Distance perpendicular to liquid surface, 0 if above surface";
  SI.Length staticHeadOutlets[M_outlets] "Distance perpendicular to liquid surface, 0 if above surface";
  SI.Pressure staticHeadInlets_Pa_relative[N_inlets] "Relative pressure to liquid surface, 0 if above surface";
  SI.Pressure staticHeadOutlets_Pa_relative[M_outlets] "Relative pressure to liquid surface, 0 if above surface";

  Real normAcc[3]=Modelica.Math.Vectors.normalize(acceleration.a) "Normalized acceleration vector (i.e. direction of acceleration)";

protected
  outer ThermofluidStream.Boundaries.AccelerationBoundary acceleration;

  SI.Pressure p_in[N_inlets] = Medium.pressure(inlet.state) "Inlets pressures";
  SI.SpecificEnthalpy h_in[N_inlets] "Inlets specific enthalpies";
  Medium.MassFraction Xi_in[Medium.nXi,N_inlets] "Inlets mass fractions";

  Medium.ThermodynamicState state_out[M_outlets] "Outlets states";
  SI.SpecificEnthalpy h_out[M_outlets] "Outlets specific enthalpies";
  Medium.MassFraction Xi_out[Medium.nXi,M_outlets] "Outlets mass fractions";

  Real d(unit="1/(m.s)") = k_volume_damping*sqrt(abs(2*L/(V*max(density_derp_h, 1e-10)))) "Friction factor for coupled boundaries";
  //SI.DerDensityByPressure density_derp_h=1e-5 "Partial derivative of density by pressure at constant specific enthalpy";
   SI.DerDensityByPressure density_derp_h=(V_ref*medium.d)/(V*K) "Partial derivative of density by pressure";

  SI.Pressure r_damping = d*der(M) "Damping of inertial pressure";

  SI.Pressure r[N_inlets] "Inertial pressure";

  SI.Temperature T_heatPort "Heat port temperature";

  SI.MassFlowRate m_flow_in[N_inlets] = inlet.m_flow "Inlets mass flow rates";
  SI.MassFlowRate m_flow_out[M_outlets] = outlet.m_flow "Outlets mass flow rates";

  final parameter Real ThresholdFull = 0.995 "Threshold value near 1 to indicate when full";
  final parameter Real ThresholdEmpty = 0.003 "Threshold value near 0 to indicate when empty";

  Boolean fFull "=true, if full (> ThresholdFull)";
  Boolean fEmpty "=true, if empty (< ThresholdEmpty)";
  Real shiftOutlet[M_outlets] "What does shiftOulet mean?";
  SI.Density liquidDensity=Medium.Liquid.density(medium.state) "Density of the liquid in the tank";
  SI.Density gasDensity=Medium.Gas.density(medium.state) "Density of the gas in the tank";

initial equation
  if initialize_pressure then
    medium.p=p_start;
  end if;

  if initialize_energy then
    if use_hstart then
      medium.h = h_start;
    else
      medium.T=T_start;
    end if;
  end if;

  if initialize_Xi then
    medium.Xi = Xi_0;
  elseif initialize_LiquidMass then
    assert(V-M_liq_start/liquidDensity > 0,"Initial liquid mass is larger then the tank can contain",level = AssertionLevel.error);
      medium.Xi={(V-M_liq_start/liquidDensity)*gasDensity/((V-M_liq_start/liquidDensity)*gasDensity+M_liq_start),M_liq_start/((V-M_liq_start/liquidDensity)*gasDensity+M_liq_start)};
  end if;

equation
  for i in 1:N_inlets loop
    assert(m_flow_in[i] > m_flow_assert, "Negative massflow at tank inlet", dropOfCommons.assertionLevel);
  end for;
    for i in 1:M_outlets loop
    assert(-m_flow_out[i] > m_flow_assert, "Positive massflow at tank outlet", dropOfCommons.assertionLevel);
  end for;

  assert(M > 0, "Tanks might not become empty");

  der(inlet.m_flow)*L = inlet.r - r - r_damping*ones(N_inlets);
  der(outlet.m_flow)*L = outlet.r - r_damping*ones(M_outlets);

  for i in 1:N_inlets loop
    staticHeadInlets_Pa_relative[i]=liquidDensity*acceleration.a*normAcc*staticHeadInlets[i];
    r[i] + p_in[i] = medium.p + max(0,staticHeadInlets_Pa_relative[i]);

    // fix potential instabilities by setting the outgoing enthalpy and mass fraction to the medium state
    h_in[i] =  if noEvent(m_flow_in[i] >= 0) then Medium.specificEnthalpy(inlet[i].state) else medium.h;
    Xi_in[:,i] = if noEvent(m_flow_in[i] >= 0) then Medium.massFraction(inlet[i].state) else medium.Xi;
  end for;
  //indication on tank nearly filled with liquid
  fFull = if V_liquid > ThresholdFull*V_ref then true else false;
  //indication on tank nearly emptied of liquid
  fEmpty = if V_liquid < ThresholdEmpty*V_ref then true else false;


  for i in 1:M_outlets loop
    // fix potential instabilities by setting the outgoing enthalpy and mass fraction to the medium state

    h_out[i] = if noEvent(-m_flow_out[i] >= 0) then Medium.specificEnthalpy(
      state_out[i]) else medium.h;
    Xi_out[:, i] = if noEvent(-m_flow_out[i] >= 0) then Medium.massFraction(
      state_out[i]) else medium.Xi;

    staticHeadOutlets_Pa_relative[i] = liquidDensity*acceleration.a*normAcc*
      staticHeadOutlets[i];

    //due to the transition interval of the reg_step, connectors must be shifted inside the tank
    //perimeters when the tank is nearly filled with liquid and there is an outlet on the surface
    //or when the tank is nearly empty and there is an outlet on the surface
    //The full case is moved further down to get better numeric performance with more gas remaining
    if fFull and abs(staticHeadOutlets[i]) < 4*outletTransition then
      shiftOutlet[i] = 3;
    elseif fEmpty and abs(staticHeadOutlets[i]) < 3*outletTransition then
      shiftOutlet[i] = -1;
    else
      shiftOutlet[i] = 0;
    end if;
    //liquid at outlet if static head >0, else gas, with smooth transition
    state_out[i] = Medium.setState_pTX(
      medium.p + max(0, staticHeadOutlets_Pa_relative[i]),
      medium.T,
      {ThermofluidStream.Undirected.Internal.regStep(
        staticHeadOutlets[i] +shiftOutlet[i]*outletTransition,
        0,
        1,
        outletTransition),ThermofluidStream.Undirected.Internal.regStep(
        staticHeadOutlets[i] + shiftOutlet[i]*outletTransition,
        1,
        0,
        outletTransition)});
  end for;


  V_liquid= MXi[end]/liquidDensity;

  medium.p = p_ref + K*(V/V_ref-1);

  der(M) = sum(inlet.m_flow) + sum(outlet.m_flow);
  der(U_med) = W_v + Q_flow + sum(inlet.m_flow.*h_in) + sum(outlet.m_flow.*h_out);
  der(MXi) = Xi_in*inlet.m_flow + Xi_out*outlet.m_flow;

  Q_flow = U*A*(T_heatPort - medium.T);
  W_v = 0;

  outlet.state = state_out;

  if not useHeatport then
    T_heatPort = medium.T;
  end if;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
       Ellipse(
         extent={{-56,76},{64,16}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={215,215,215},
         fillPattern=FillPattern.Solid,
         pattern=LinePattern.None),
       Rectangle(
         extent={{-56,46},{64,-56}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={215,215,215},
         fillPattern=FillPattern.Solid,
         pattern=LinePattern.None),
       Ellipse(
         extent={{-56,-28},{64,-88}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={215,215,215},
         fillPattern=FillPattern.Solid,
         pattern=LinePattern.None),
       Line(
         points={{-100,0},{100,0}},
         color={28,108,200},
         thickness=0.5),
       Ellipse(
         extent={{-60,-20},{60,-80}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={170,213,255},
         fillPattern=FillPattern.Solid),
       Rectangle(
         extent={{-60,50},{60,-50}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={170,213,255},
         fillPattern=FillPattern.Solid,
         pattern=LinePattern.None),
       Ellipse(
         extent={{-60,80},{60,20}},
         lineColor={28,108,200},
         lineThickness=0.5,
         fillColor={170,213,255},
         fillPattern=FillPattern.Solid),
       Line(
         points={{-60,50},{-60,-52}},
         color={28,108,200},
         thickness=0.5),
       Line(
         points={{60,50},{60,-52}},
         color={28,108,200},
         thickness=0.5),
        Ellipse(extent={{-60,24},{60,-32}}, lineColor={28,108,200}),
        Line(
          points={{20,100},{-24,42}},
          color={206,103,0},
          arrow={Arrow.None,Arrow.Filled},
          thickness=1),
        Line(
          points={{52,78},{8,20}},
          color={206,103,0},
          arrow={Arrow.None,Arrow.Filled},
          thickness=1),
        Line(
          points={{84,56},{40,-2}},
          color={206,103,0},
          arrow={Arrow.None,Arrow.Filled},
          thickness=1),
       Text(
         extent={{-120,60},{-82,20}},
         textColor={116,116,116},
          textString="%N_inlets"),
       Text(
         extent={{80,60},{120,20}},
         textColor={116,116,116},
          textString="%M_outlets"),
        Text(
          extent={{40,102},{84,70}},
          textColor={206,103,0},
          textString="a")}),Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>This Volume is the parent class for Accumulator and Receiver models that separate the two phases and are able to output gas, liquid or two-phase medium, depending on its liquid level and the height of the outlet. </p>
<p>To complete the partialTank, equations for total tank volume V, centreOfMass, staticHeadInlets and staticHeadOutlets needs to be provided. This separation is made to make it easy to implement arbitrary geometries. In this component, medium.p is interpreted as the pressure at the liquid surface.</p>
<p>Since there is no formula to compute density_derp_h for this volume, an upper bound has to be set in the parameter density_derp_h_set. Alternativeley the derivative can be taken from the media model for all the media that implement the corresponding formula by setting density_derp_h_from_media=true (default:false)</p>
<p><span style=\"color: #ff5500;\">Beware: This is a new addition to the library. It may be subject to design reconsiderations in future versions.</span></p>
</html>", revisions="<html>
<p><img src=\"modelica:/ThermofluidStream/Resources/saab_logo.png\"/>Author: Ingela Lind, M Sc, Ph D, Technical Fellow,
Simulation and Thermal Analysis,
Vehicle Systems,
SAAB Aerosystems, 2024
</p>
</html>"));
end PartialTank;
