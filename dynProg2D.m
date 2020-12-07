function [costToGo,nextX,nextY]=dynProg2D(seriesHybrid,tVec,discX,discY,J_T);
  
% dynProg2DS - Dynamic programming for 2 states with arc costs as a matrix.
%
% [cost,pathX,pathY]=dynProg2D(funName,tVec,discX,discY,J_T);
%
% dynProg2DS - Solves a dynamic programming problem for two state variables, supporting a
% matrix loss function implementation. This function will call the user function
% "funName", given as a function handle input, to compute the scalar arc cost of going
% from one state (x_i(t),y_i(t)) to all other possibles states (x_j(t+1),y_j(t+1)). The
% function call is made like this:
%
%     funName([tVec(t) tVec(t+1)],discX(ii),discX,discY(jj),discY)
%
% The resulting optimal trajectory and optimal costs are returned in the 3-dimensional
% matrices pathX, pathY, and cost.
%
% It is assumed that the discretization is independent of time, but the arcs may be a
% function of time, as they are implemented in the user function "funName", given as a
% function handle input.
%
% Inputs:
%  funName   - Function handle to the loss function.
%  tVec      - Vector with the time step discretization.
%  discX     - Vector with x-discretization.
%  discY     - Vector with y-discretization.
%  J_T       - Final costs associated with the final states at t=T.
%
% Outputs:
%  cost      - Cost matrix, showing the cost-to-go from each state (3-dim).
%  pathX     - Indices showing the best path in the x-variables (3-dim).
%  pathY     - Indices showing the best path in the y-variables (3-dim).
  
% (C) 2008 Lars Eriksson
% 1.0 First version 2008-04-10
% 1.1  2008-08-25 Corrected bug in squeeze(costToGo(t+1,:,:))


  nextX=zeros(length(tVec),length(discX),length(discY));
  nextY=zeros(length(tVec),length(discX),length(discY));
  costToGo=zeros(length(tVec),length(discX),length(discY));
  costToGo(end,:,:)=squeeze(costToGo(end,:,:))+J_T;
  % Outer loop over time
  for t=(length(tVec)-1):-1:1,
    % Inner loops over the grid
    for ii=1:length(discX),
      for jj=1:length(discY),
        % We are now in one point in the grid.
        % Calculate the costs for all arcs from this point
        currCost=seriesHybrid([tVec(t) tVec(t+1)],discX(ii),discX,discY(jj),discY)+squeeze(costToGo(t+1,:,:));
        % Find the arc that has the minimum cost
        [val,II]=min(currCost);
        [val,JJ]=min(val);
        % Store the best arc
        II=II(JJ);
        nextX(t,ii,jj)=II;
        nextY(t,ii,jj)=JJ;
        % Store the associated
        costToGo(t,ii,jj)=currCost(II,JJ);
      end
    end
  end
  