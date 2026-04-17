pub fn gauss_sediel(a: &Vec<Vec<f64>>, b: &Vec<f64>, x: &mut [f64; 6], iterations: usize) {
    let n = b.len();

    for _ in 0..iterations {
        let mut completed: bool = true;

        for i in 0..n {
            let mut sum = 0.0;
            let x_old = x[i];

            for j in 0..n {
                if i != j {
                    sum += a[i][j] * x[j];
                }
            }
            x[i] = (b[i] - sum) / a[i][i];

            if ((x[i] - x_old).abs() / x[i]) * 100.0 > 1e-5 {
                completed = false;
            };
        }

        if completed {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_3_3() {
        let a = vec![
            vec![4.0, 1.0, 2.0],
            vec![1.0, 5.0, 1.0],
            vec![2.0, 1.0, 6.0],
        ];

        let b = vec![16.0, 15.0, 24.0];

        let mut x = [0.0; 6];

        gauss_sediel(&a, &b, &mut x, 200);

        assert!(
            (x[0] - 2.0).abs() < 1e-6,
            "{} expected, found {}",
            2.0,
            x[0]
        );
        assert!(
            (x[1] - 2.0).abs() < 1e-6,
            "{} expected, found {}",
            2.0,
            x[1]
        );
        assert!(
            (x[2] - 3.0).abs() < 1e-6,
            "{} expected, found {}",
            3.0,
            x[2]
        );
    }

    #[test]
    fn maximum_test() {
        let a = vec![
            vec![8.0, 1.0, 2.0, 0.0, 0.0, 0.0],
            vec![1.0, 10.0, 1.0, 3.0, 0.0, 0.0],
            vec![2.0, 1.0, 12.0, 1.0, 2.0, 0.0],
            vec![0.0, 3.0, 1.0, 14.0, 1.0, 1.0],
            vec![0.0, 0.0, 2.0, 1.0, 16.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 18.0],
        ];

        let b = vec![16.0, 36.0, 54.0, 76.0, 96.0, 117.0];

        let mut x = [0.0; 6];

        gauss_sediel(&a, &b, &mut x, 30);

        assert!((x[0] - 1.0).abs() < 1e-6);
        assert!((x[1] - 2.0).abs() < 1e-6);
        assert!((x[2] - 3.0).abs() < 1e-6);
        assert!((x[3] - 4.0).abs() < 1e-6);
        assert!((x[4] - 5.0).abs() < 1e-6);
        assert!((x[5] - 6.0).abs() < 1e-6);
    }
}
