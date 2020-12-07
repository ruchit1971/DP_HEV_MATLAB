function [costToGo,next]=dynProg1D(parallelHybrid,tVec,disc,J_T);
  
% dynProg1D - Dynamic programming for 1 state with arc costs as a matrix.
%
% [cost,path]=dynProg1D(funName,tVec,disc,J_T);
% 
% Solves a dynamic programming problem for one state variable, supporting a matrix loss
% function implementation. This function will call the user function "funName", given as a
% function handle input, to compute the scalar arc cost of going from one state
% (x_i(t)) to all another possible states (x(t+1)). The function call is
% made like this:
%
%     funName([tVec(t) tVec(t+1)],disc(ii),disc)
%
% The resulting optimal trajectory and optimal costs are returned in the 2-dimensional
% matrices path, and cost.
%
% It is assumed that the discretization is independent of time, but the arcs may be a
% function of time, as they are implemented in the user function "funName", given as a
% function handle input.
%
% Inputs:
%  funName   - Function handle to the loss function.
%  tVec      - Vector with the time step discretization.
%  disc      - Vector with state discretization.
%  J_T       - Final costs associated with the final states at t=T.
%
% Outputs:
%  cost      - Cost matrix, showing the cost-to-go from each state (2-dim).
%  path      - Indices showing the best path in the state variables (2-dim).
  
% (C) 2008 Lars Eriksson
% 1.0 First version 2008-04-10
% 1.1  2008-08-25 Corrected bug in costToGo(t+1,:)



  next=zeros(length(tVec),length(disc));
  costToGo=zeros(length(tVec),length(disc));
  costToGo(end,:)=costToGo(end,:)+J_T;
  % Outer loop over time
  for t=(length(tVec)-1):-1:1,
    % Inner loops over the grid
    for ii=1:length(disc),
      % We are now in one point in the grid.
      % Calculate the cost for all arcs from this point
      coost=parallelHybrid([tVec(t) tVec(t+1)],disc(ii),disc);
      currCost=coost+costToGo(t+1,:);
      % Find the arc that has the minimum cost
      [val,JJ]=min(currCost);
      % Store the best arc
      next(t,ii)=JJ;
      % Store the associated cost
      costToGo(t,ii)=currCost(JJ);
    end
  end

  
  
