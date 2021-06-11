void AnymalKinoCentroidalDynamicsAD_systemFlowMap_jacobian_sparsity(unsigned long const** row,
                                                                    unsigned long const** col,
                                                                    unsigned long* nnz) {
   static unsigned long const rows[300] = {0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,12,13,14,15,16,17,18,19,20,21,22,23};
   static unsigned long const cols[300] = {24,27,30,33,25,28,31,34,26,29,32,35,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,25,26,28,29,31,32,34,35,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,26,27,29,30,32,33,35,6,7,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,27,28,30,31,33,34,0,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,1,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,36,37,38,39,40,41,42,43,44,45,46,47,36,37,38,39,40,41,42,43,44,45,46,47};
   *row = rows;
   *col = cols;
   *nnz = 300;
}
