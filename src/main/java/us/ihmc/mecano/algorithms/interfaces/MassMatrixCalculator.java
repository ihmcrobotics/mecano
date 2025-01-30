package us.ihmc.mecano.algorithms.interfaces;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public interface MassMatrixCalculator
{

   void compute();

   DMatrixRMaj getMassMatrix();

   void getMassMatrix(DMatrixRMaj massMatrixToPack);

   JointBasics[] getJointsInOrder();
}