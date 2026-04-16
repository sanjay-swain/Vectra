pub fn gauss_sediel(a: &Vec<Vec<f64>>, b: &Vec<f64>, x: &mut [f64; 6], iterations: usize) {
    let mut iter: usize = 0;
    let n = b.len();

    while iter < iterations {
        let mut i: usize = 0;
        while i < n {
            let mut inner_sum: f64 = 0.0;
            let mut j: usize = 0;
            while j < n {
                if j != i {
                    inner_sum = inner_sum + a[i][j] * x[j];
                }
                j = j + 1;
            }
            x[i] = (b[i] - inner_sum) / a[i][i];
            i = i + 1;
        }

        iter = iter + 1;
    }
}
