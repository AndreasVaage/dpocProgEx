function  compareG(G1,G2)
diff = G1 - G2;
[i,j] = ind2sub(size(diff),find(not(isnan(diff)) & (diff > 0.000001 | diff < -0.000001)));
disp('Not equal at:')
disp('  State Controlinput')
disp([i j])
end

