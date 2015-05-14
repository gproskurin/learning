type int_t = i64;

struct Job {
	w : int_t,
	l : int_t,
}

fn weighted_sum(jobs : &[Job], idx : &[usize]) -> int_t
{
	let mut end_time = 0;
	let mut wsum = 0;
	let mut cur = 0;
	for i in idx {
		let j : &Job = &jobs[*i];
		end_time += j.l;
		//println!("job {}, idx:{} w:{} l:{} end:{} delta_wsum:{}", cur, i, j.w, j.l, end_time, (j.w * end_time));
		wsum += j.w * end_time;
		cur += 1;
	}
	return wsum;
}

fn load_jobs() -> Vec<Job>
{
	use std::sync::mpsc::{SyncSender, Receiver};
	let (tx, rx) : (SyncSender<String>, Receiver<String>) = std::sync::mpsc::sync_channel(0);
	std::thread::spawn(move || {
		use std::io;
		use std::io::prelude::*;
		let stdin = std::io::stdin();
		for line in stdin.lock().lines() {
			let l = line.unwrap();
			if l.is_empty() {
				break;
			}
			tx.send(l).unwrap();
		}
		drop(tx);
	});

	let mut res = Vec::<Job>::new();
	let num : usize = rx.recv().unwrap().trim().parse::<usize>().unwrap();
	res.reserve(num);
	loop {
		let data = rx.recv();
		if data.is_err() {
			break;
		}
		let s = data.unwrap();
		let v: Vec<&str> = s.split(|c: char| c.is_whitespace()).collect();
		assert!(v.len()==2);
		let w = v[0].parse::<int_t>().unwrap();
		let l = v[1].parse::<int_t>().unwrap();
		res.push(Job {w:w,l:l});
	}
	assert!(res.len() == num);
	return res;
}


fn run(jobs : &[Job])
{
	let mut idx1 : Vec<usize> = Vec::new();
	idx1.reserve(jobs.len());

	for i in 0..jobs.len() {
		idx1.push(i);
	}

	let mut idx2 = idx1.clone();

	println!("wsum0(orig): {}", weighted_sum(jobs, &idx1));

	idx1.sort_by(
		|i1,i2| {
			use std::cmp::Ordering;
			let j1 : &Job = &jobs[*i1];
			let j2 : &Job = &jobs[*i2];
			let wl1 = j1.w - j1.l;
			let wl2 = j2.w - j2.l;
			if wl1 > wl2 {
				return Ordering::Less;
			}
			if wl1 < wl2 {
				return Ordering::Greater;
			}
			if j1.w > j2.w {
				return Ordering::Less;
			}
			return Ordering::Greater;
		}
	);

	println!("wsum1(diff): {}", weighted_sum(jobs, &idx1));

	idx2.sort_by(
		|i1,i2| {
			use std::cmp::Ordering;
			let j1 : &Job = &jobs[*i1];
			let j2 : &Job = &jobs[*i2];
			let w1l2 = j1.w * j2.l;
			let w2l1 = j2.w * j1.l;
			if w1l2 > w2l1 {
				return Ordering::Less;
			}
			if w1l2 < w2l1 {
				return Ordering::Greater;
			}
			if j1.w > j2.w {
				return Ordering::Less;
			}
			return Ordering::Greater;
		}
	);
	println!("wsum2(div) : {}", weighted_sum(jobs, &idx2));
}

fn main()
{
	let jobs = load_jobs();
	run(&jobs);
}
