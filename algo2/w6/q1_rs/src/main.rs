#![feature(scoped)]

extern crate rand;
extern crate num_cpus;


#[derive(Copy,Clone)]
struct Literal {
	negate : bool,
	varnum : usize,
}

type Clause = Vec<Literal>; // XXX array[2]
type Expr = Vec<Clause>;

type Ass = Vec<bool>;
type OptAss = Option<Ass>;
type FixedAss = std::collections::HashMap<usize, bool>;

fn calc_l(l : &Literal, ass : &Ass) -> bool
{
	let var : bool = ass[l.varnum];
	let res1 : bool = if l.negate { !var } else { var };
	let res2 : bool = var ^ l.negate;
	assert!(res1 == res2);
	return res1;
}

fn calc_c(c : &Clause, ass : &Ass) -> bool
{
	// OR
	for l in c {
		let l_val : bool = calc_l(l, ass);
		if l_val == true {
			return true;
		}
	}
	return false;
}

fn calc_e(e : &Expr, ass : &Ass) -> bool
{
	// AND
	for c in e {
		let c_val : bool = calc_c(c, ass);
		if c_val == false {
			return false;
		}
	}
	return true;
}

fn calc_e_get_falses(e : &Expr, ass : &Ass) -> Vec<usize>
{
	let mut falses = Vec::new();
	for i in 0..e.len() {
		let c : &Clause = &e[i];
		let c_val : bool = calc_c(c, ass);
		if c_val == false {
			falses.push(i);
		}
	}
	return falses;
}

fn get_rnd_usize() -> usize
{
	use rand::Rng;
	let mut rng = rand::thread_rng();
	let r : usize = rng.gen();
	return r;
}

fn get_rnd_bool() -> bool
{
	use rand::Rng;
	let mut rng = rand::thread_rng();
	let r : bool = rng.gen();
	return r;
}

fn gen_rnd_ass(sz : usize, fix_ass : &FixedAss) -> Ass
{
	let mut ass = Ass::new();
	ass.reserve(sz);
	for vn in 0..sz {
		let b : bool = match fix_ass.get(&vn) {
			Some(&val) => { val },
			None => get_rnd_bool()
		};
		ass.push(b);
	}
	assert!(ass.len()==sz);
	return ass;
}

fn print_ass_stat(ass : &Ass)
{
	let mut count_true : usize = 0;
	let mut count_false : usize = 0;
	for b in ass {
		if *b {
			count_true += 1;
		} else {
			count_false += 1;
		}
	}
	assert!(count_false+count_true == ass.len());
	println!("ass size:{} false:{} true:{}", ass.len(), count_false, count_true);
}

fn read_clause() -> Clause
{
	let mut c_res = Clause::new();

	let mut line = String::new();
	std::io::stdin().read_line(& mut line).ok();
	let line = line.trim();
	let mut c_str_v : Vec<&str> = line.trim().split(|c:char| c.is_whitespace()).collect();

	for l_str in c_str_v {
		let n : isize = l_str.parse().unwrap();
		let l = {
			if n < 0 {
				let num : usize = (-n) as usize;
				assert!(num >= 1);
				Literal { varnum:num-1, negate:true }
			} else {
				let num : usize = n as usize;
				assert!(num >= 1);
				Literal { varnum:num-1, negate:false }
			}
		};
		c_res.push(l);
	}
	assert!(c_res.len()==2);
	return c_res;
}

fn load_expr() -> (Expr, usize)
{
	let mut res = Expr::new();

	let mut line = String::new();
	std::io::stdin().read_line(& mut line).ok();
	let V : usize = line.trim().parse::<usize>().unwrap();

	println!("V:{}", V);
	res.reserve(V);
	for _ in 0..V {
		res.push(read_clause());
	}

	return (res, V);
}

fn print_stat(e : &Expr)
{
	println!("Expr size: {}", e.len());
	if (!e.is_empty()) {
		let mut min_var : usize = e[0][0].varnum;
		let mut max_var : usize = min_var;
		for c in e {
			for l in c {
				if l.varnum > max_var {
					max_var = l.varnum;
				}
				if l.varnum < min_var {
					min_var = l.varnum;
				}
			}
		}
		println!("min_var:{} max_var:{}", min_var, max_var);
	}
}

/*
class my_log {
	std::ostream& os_;
	std::ostringstream os_str_;
public:
	my_log(std::ostream& os) : os_(os) {}
	template <typename T>
	std::ostream& operator<<(const T& t) {
		return os_str_ << t;
	}
	~my_log() {
		os_ << os_str_.str() << std::flush;
	}
};
*/

fn expr_simplify(mut e : Expr, V : usize) -> (Expr, FixedAss)
{
	type VarsSet = std::collections::HashSet<usize>;

	let mut fix_ass = FixedAss::new();

	loop {
		let mut improved = false; // did something for optimizationn?

		let mut pos = VarsSet::new();
		let mut neg = VarsSet::new();

		// collect usage of vars
		for c in &e {
			for l in c {
				assert!(l.varnum < V);
				if l.negate {
					neg.insert(l.varnum);
				} else {
					pos.insert(l.varnum);
				}
			}
		}

		// Fix unique variables.
		// If variable does not appear in expression at all, fix its value to something (value doesn't matter)
		for vn in 0..V {
			if !pos.contains(&vn) && !neg.contains(&vn) && !fix_ass.contains_key(&vn) {
				fix_ass.insert(vn, true);
				//fix_ass.emplace(vn, false);
				//fix_ass.emplace(vn, get_rnd_bool(nullptr));
				improved = true;
			}
		}

		let mut new_e = Expr::new();
		for c in &e {
			let mut copy_cur_clause = true;
			for l in c {
				let vn = &l.varnum;

				assert!(pos.contains(vn) || neg.contains(vn));

				// all occurrences are negative (or none), fix var to be false
				if !pos.contains(vn) {
					assert!(neg.contains(vn));
					fix_ass.insert(*vn, false);
					copy_cur_clause = false;
					break;
				}

				// all occurrences are positive (or none), fix var to be true
				if !neg.contains(vn) {
					fix_ass.insert(*vn, true);
					copy_cur_clause = false;
					break;
				}
			}
			if copy_cur_clause {
				new_e.push(c.clone());
			} else {
				improved = true; // expression become smaller
			}
		}

		assert!(e.len() >= new_e.len());
		if new_e.len() < e.len() {
			assert!(improved);
		}

		if !improved {
			return (new_e, fix_ass); // XXX move
		}
		e = new_e.clone();
	} // loop
}

fn papadimitriou_inner(V : usize, e : &Expr, fa : &FixedAss, inner_iter : usize) -> OptAss
{
	let mut ass : Ass = gen_rnd_ass(V, fa);

	for j in 0..inner_iter {
		let falses = calc_e_get_falses(e, &ass);
		if falses.is_empty() {
			//print_ass_stat(ass);
			assert!(calc_e(e, &ass) == true);
			//my_log(std::cout) << "TRUE???\n";
			println!("TRUE???");
			return Some(ass);
		}

		// asserts
		for f_pos in &falses {
			let false_c : &Clause = &e[*f_pos];
			assert!(calc_c(&false_c, &ass) == false);
			for l in false_c {
				assert!(calc_l(l, &ass) == false);
				assert!(!fa.contains_key(&l.varnum));
			}
		}

		if j % 100000 == 0 {
			//my_log(std::cout)
			//	<< "iter:" << j << "/" << inner_iter
			//	<< " false_clauses:" << falses.size() << "\n";
		}

		// negate variables for random false clause
		// sometimes not negate, but assign random values
		let rnd : usize = get_rnd_usize() % falses.len();
		let false_c_pos : usize = falses[rnd];
		let false_c : &Clause = &e[false_c_pos];
		for l in false_c {
			assert!(!fa.contains_key(&l.varnum));
			if get_rnd_bool() {
				ass[l.varnum] = !ass[l.varnum];
			} else {
				ass[l.varnum] = get_rnd_bool();
			}
		}
	}
	return None;
}

fn papadimitriou_parallel(ass_sz : usize, e : &Expr, fa : &FixedAss) -> OptAss
{
	let outer_iter : usize = num_cpus::get();; // threads count
	let inner_iter : usize = 1000000;
	let mut fut = Vec::new();
	for _ in 0..outer_iter {
		let f = std::thread::scoped(|| papadimitriou_inner(ass_sz, &e, &fa, inner_iter) );
		fut.push(f);
	}

	println!("THREADS: {}", fut.len());

	let mut res : OptAss = None;
	for f in fut {
		let cur_res = f.join();
		match res {
			None => { res = cur_res; }
			_ => {}
		}
	}
	return res;
}

fn run()
{
	let (e, V) = load_expr();
	print_stat(&e);
	println!("");

	let (opt_e, fa) : (Expr, FixedAss) = expr_simplify(e.clone(), V);

	println!("vars optimized away (fixed ass): {} remains:{}", fa.len(), V - fa.len());
	println!("");


	let res = papadimitriou_parallel(V, &opt_e, &fa);

	match res {
		Some(ass) => {
			let r : bool = calc_e(&e, &ass);
			if r==false {
				panic!("BAD_PROGRAMMER_2");
			} else {
				println!("Result: TRUE!!!");
				print_ass_stat(&ass);
			}
		},
		None => println!("Result: may be false...")
	}
}

fn main()
{
	run();
}
