public class PercolationStats {

	private double[] fractions;
	private final int N;
	private final int T;

	private double myMean = -1;
	private double mySigma = -1;
	private double confLo = -1;
	private double confHi = -1;

	public PercolationStats(int n, int t)	 // perform T independent experiments on an N-by-N grid
	{
		if (n <= 0 || t <= 0)
			throw new IllegalArgumentException();
		N = n;
		T = t;
		fractions = new double[T];
		int frEnd = 0;
		for (int i = 0; i < T; ++i) {
			//System.out.println(""+i+"/"+T);
			Percolation p = new Percolation(N);
			int openedCur = 0;
			while (!p.percolates()) {
				final int r = StdRandom.uniform(N)+1;
				final int c = StdRandom.uniform(N)+1;
				if (!p.isOpen(r, c)) {
					p.open(r, c);
					++openedCur;
				}
			}
			fractions[frEnd++] = (double) openedCur / N / N;
		}
		assert frEnd == fractions.length;
	}

	public double mean()					  // sample mean of percolation threshold
	{
		if (myMean < 0) {
			double frSum = 0;
			for (double f : fractions) {
				frSum += f;
			}
			myMean = frSum / T;
		}
		assert myMean >= 0;
		return myMean;
	}

	public double stddev()					// sample standard deviation of percolation threshold
	{
		if (mySigma < 0) {
			double sumDiffSq = 0;
			for (double f : fractions) {
				double diff = f - mean();
				sumDiffSq += diff*diff;
			}
			mySigma = Math.sqrt(sumDiffSq / (T-1));
		}
		assert mySigma >= 0;
		return mySigma;
	}

	public double confidenceLo()			  // low  endpoint of 95% confidence interval
	{
		if (confLo < 0) {
			confLo = mean() - 1.96 * stddev() / Math.sqrt(T);
		}
		assert confLo >= 0;
		return confLo;
	}

	public double confidenceHi()			  // high endpoint of 95% confidence interval
	{
		if (confHi < 0) {
			confHi = mean() + 1.96 * stddev() / Math.sqrt(T);
		}
		assert confHi >= 0;
		return confHi;
	}

	public static void main(String[] args)	// test client (described below)
	{
		if (args.length != 2) {
			System.err.println("args");
			return;
		}
		final int NN = Integer.parseInt(args[0]);
		final int TT = Integer.parseInt(args[1]);
		PercolationStats ps = new PercolationStats(NN, TT);
		System.out.println("mean: " + ps.mean());
		System.out.println("sigma: " + ps.stddev());
		System.out.println("confLo: " + ps.confidenceLo());
		System.out.println("confHi: " + ps.confidenceHi());
	}
}
