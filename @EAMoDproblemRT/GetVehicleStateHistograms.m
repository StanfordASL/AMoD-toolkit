function [ChargingVehicleHist,DischargingVehicleHist,PaxVehicleHist,...
    RebVehicleHist,IdleVehicleHist,AllVehicleHist] ...
    = GetVehicleStateHistograms(obj,varargin)
% GetVehicleStateHistograms Overrides AbstractEAMoDproblem.GetVehicleStateHistograms to include pre-routed customer-carrying vehicles
%
%   See also AbstractEAMoDproblem.GetVehicleStateHistograms

if obj.source_relax_flag
   warning('Using the real-time formulation with source_relax_flag set is not recommended: vehicles are not necessarily conserved in the sense of VehicleConservationTest.') 
end

[ChargingVehicleHist,DischargingVehicleHist,~,...
    RebVehicleHist,IdleVehicleHist] ...
    = GetVehicleStateHistograms@AbstractEAMoDproblem(obj,varargin{:});

PaxVehicleHist = obj.GetPreRoutedTripHistogram();

AllVehicleHist = ChargingVehicleHist + DischargingVehicleHist + PaxVehicleHist + RebVehicleHist + IdleVehicleHist;
end