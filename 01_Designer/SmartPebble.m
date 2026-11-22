% This file contains the smart pebble class with all relevant members
% describing a smart pebble's properties.

classdef SmartPebble
   properties
	   % a = 0.47*0.03 % E1
	   % b = 0.94*0.03
	   % a = 0.67*0.03 % E5 
	   % b = 0.68*0.03
	   a = 0.012 % EP
	   b = 0.021
	   c = 0.03
	   y1
	   y2
	   y3
	   rho_desired = 2526
	   rho_ballast = 8800
	   rho_resin = 1137
	   cavity = [0.012,0.020,0.030]
	   vol
	   shell_thickness = 0.001
	   mass
   end
   methods
	   function obj = SmartPebble()
		   obj.y1 = obj.a/obj.c;
		   obj.y2 = obj.b/obj.c;
		   obj.y3 = obj.c;
		   obj.vol = 4/3*pi*obj.a*obj.b*obj.c;
		   obj.mass = 4/3*pi*obj.a*obj.b*obj.c*obj.rho_desired;
	  end
   end
end
