const double to_deg = 180.0 / M_PI;
const double to_rad = M_PI / 180.0;

class Point {
public:
	double x, y;

	Point& operator+=(const Point& rhs) {x += rhs.x; y += rhs.y; return *this;}
	Point& operator-=(const Point& rhs) {x -= rhs.x; y -= rhs.y; return *this;}
	Point& operator*=(double rhs) {x *= rhs; y *= rhs; return *this;}
	Point& operator/=(double rhs) {x /= rhs; y /= rhs; return *this;}
	friend Point operator+(const Point& lhs, const Point& rhs) {
		Point tmp(lhs);
		return (tmp += rhs);
	}
	friend Point operator-(const Point& lhs, const Point& rhs) {
		Point tmp(lhs);
		return (tmp -= rhs);
	}
	friend double operator*(const Point& lhs, const Point& rhs) {  // dot product
		return (lhs.x * rhs.x + lhs.y * rhs.y);
	}
	friend Point operator*(const Point& lhs, double rhs) {  // scalar product
		Point tmp(lhs);
		return (tmp *= rhs);
	}
	friend Point operator/(const Point& lhs, double rhs) {
		Point tmp(lhs);
		return (tmp /= rhs);
	}
	double getPhase() const {
		return atan2(y, x);
	}

    double dist2(const Point& other) {
        const double dx = x - other.x;
        const double dy = y - other.y;
        return dx*dx + dy*dy;
    }
};

//#define DEBUG

enum EntityType { WIZARD, SNAFFLE, BLUDGER, POLE };

struct WizardExtra {
	int_fast16_t team, snaffle, timeout;
};

struct SnaffleExtra {
	int_fast16_t alive, carrier;
};

struct BludgerExtra {
	int_fast16_t last, ignores;
};

union EntityExtra {
	WizardExtra wi;
	SnaffleExtra sn;
	BludgerExtra bl;
};

class Entity {
	public:
		Point p, v;
		double r, m, friction;
		EntityType type;
		EntityExtra e;

		void apply_force(const Point& direction, double power);
		void move(double dt);
		void end_turn();

		double sub_collide(const Entity& other, bool debug) const;
		void do_collide(Entity& other);
        void do_collide_pole(const Entity& other);

		void make_wizard(const Point& pos, int_fast16_t team);
		void make_snaffle(const Point& pos);
		void make_bludger(const Point& pos);
};

void Entity::make_wizard(const Point& pos, int_fast16_t team) {
	type = WIZARD;
	p = pos;
	v.x = 0;
	v.y = 0;
	r = 400.0;
	m = 1.0;
	friction = 0.75;
	e.wi.team = team;
	e.wi.snaffle = -1;
	e.wi.timeout = 0;
}

void Entity::make_snaffle(const Point& pos) {
	type = SNAFFLE;
	p = pos;
	v.x = 0;
	v.y = 0;
	r = 150.0;
	m = 0.5;
	friction = 0.75;
	e.sn.alive = 1;
	e.sn.carrier = -1;
}

void Entity::make_bludger(const Point& pos) {
	type = BLUDGER;
	p = pos;
	v.x = 0;
	v.y = 0;
	r = 200;
	m = 8.0;
	friction = 0.9;
	e.bl.last = -1;
	e.bl.ignores = 0;
}

/*
ostream& operator<<(ostream& os, const Entity& obj) {
	os << "p(" << obj.p.x << ", " << obj.p.y << ") v(" << obj.v.x << ", " << obj.v.y << ") at " << obj.angle;
	os << " in (" << obj.cur_lap << ", " << obj.ncpid;
	Point d_vec = g.cps[obj.ncpid] - obj.p;
	os << ") with distance " << sqrt(d_vec*d_vec);
	return os;
}
*/

inline void Entity::apply_force(const Point& direction, double power) {
	const double norm = sqrt(direction * direction);
	if (norm > 0) {
		Point vec = direction / norm;
		vec *= power / m;
		v += vec;
	}
}

inline void Entity::move(double dt) {
    if (type != SNAFFLE || e.sn.carrier == -1)
        p += v * dt;
}

inline void Entity::end_turn() {
	p.x = round(p.x);
	p.y = round(p.y);
	v.x = round(v.x * friction);
	v.y = round(v.y * friction);
}

double Entity::sub_collide(const Entity& other, bool debug) const {
	if (other.type == SNAFFLE && other.e.sn.carrier != -1) {
#ifdef DEBUG
		if (debug) cerr << "No collision, because snaffle is already carried" << endl;
#endif
		return -1; // carried snaffle can't collide
	}
	if (type == WIZARD && (e.wi.snaffle >= 0 || e.wi.timeout > 0) && other.type == SNAFFLE) {
#ifdef DEBUG
		if (debug) cerr << "No collision, because wizard not ready for snaffle pickup" << endl;
#endif
		return -1; // wizard not ready
	}
	double r_square = (r + other.r);
	if (type == WIZARD && other.type == SNAFFLE) {
		r_square -= 151.0;
	}
	r_square = r_square * r_square;
	const Point dp = p - other.p;
	const Point dv = v - other.v;
	const double b = dp * dv;
#ifdef DEBUG
    if (debug) {
        cerr << "Collision check: pos " << p.x << " " << p.y << " and " << other.p.x << " " << other.p.y << endl;
        cerr << "  vel " << v.x << " " << v.y << " and " << other.v.x << " " << other.v.y << endl;
        cerr << " Dp " << dp.x << " " << dp.y << endl;
        cerr << " Dv " << dv.x << " " << dv.y << endl;
        cerr << "  B " << b << endl;
    }
#endif
	if (b >= 0 && (type != WIZARD || other.type != SNAFFLE)) {
#ifdef DEBUG
		if (debug) cerr << "No collision, because objects move away from each other" << endl;
#endif
		return -1; // b > 0 [-> objects moving away from each other
	}
	const double c = dp*dp - r_square;
    //if (c <= 0 && (c > -2.0 || type == WIZARD && other.type == SNAFFLE)) {
    if (c <= 0) {
#ifdef DEBUG
		if (debug) cerr << "Immediate collision because of intersection" << endl;
#endif
		return 0; // c <= 0 [-> distance critical low]
	}
	const double a = dv * dv;
	//if (a == 0) return -1;
	const double det = b*b - a*c;
#ifdef DEBUG
    if (debug) {
        cerr << "  C " << c << endl;
        cerr << "  A " << a << endl;
        cerr << " det " << det << endl;
    }
#endif
	if (det >= 0) {
		double t1 = (-b - sqrt(det)) / a;
		if (0 <= t1)  // XXX also return t1 > 1 for simple collision prediction?
			return t1; // with b<0, c>0 and det>0 the test should always be true now
	}
#ifdef DEBUG
	if (debug) cerr << "No collision because paths don't intersect " << det << endl;
#endif
	return -1;
}

void Entity::do_collide(Entity& other) {
#ifdef DEBUG
	cerr << "Collision resolution: old vel " << v.x << " " << v.y << " and " << other.v.x << " " << other.v.y << endl;
	cerr << "  current position " << p.x << " " << p.y << " and " << other.p.x << " " << other.p.y << endl;
#endif
	double r_square = (r + other.r);
	r_square = r_square * r_square;
	const double m1 = m;
	const double m2 = other.m;
	const double mc =  (m1/m2 + 1) / m1;
	const Point dp = p - other.p;
	const Point dv = v - other.v;
	Point impact = dp * (dv*dp / (r_square * mc) * 2);
#ifdef DEBUG
	cerr << "  m1:" << m1 << " m2:" << m2 << " mc:" << mc << " dp: " << dp.x << " " << dp.y;
	cerr << " dv: " << dv.x << " " << dv.y << endl;
#endif
    const double impact_norm = sqrt(impact*impact);
#ifdef DEBUG
    cerr << " giving impact " << impact.x << " " << impact.y << " with norm " << impact_norm << endl;
#endif
    if (impact_norm < 200.0) {
        impact *= 0.5 + 100.0 / impact_norm;
#ifdef DEBUG
        cerr << " adjusting norm, now impact is " << impact.x << " " << impact.y << endl;
#endif
	}
	v -= impact / m1;
    other.v += impact / m2;
#ifdef DEBUG
	cerr << "  new vel " << v.x << " " << v.y << " and " << other.v.x << " " << other.v.y << endl;
#endif
}

void Entity::do_collide_pole(const Entity& other) {
#ifdef DEBUG
	cerr << "Collision resolution: old vel " << v.x << " " << v.y << " and " << other.v.x << " " << other.v.y << endl;
	cerr << "  current position " << p.x << " " << p.y << " and " << other.p.x << " " << other.p.y << endl;
#endif
    double r_square = (r + other.r);
    r_square = r_square * r_square;
	const double m1 = m;
	const double mc = 1 / m1;
	const Point dp = p - other.p;
	const Point dv = v - other.v;
	Point impact = dp * (dv*dp / (r_square * mc) * 2);
#ifdef DEBUG
	cerr << "  m1:" << m1 << " m2:" << INFINITY << " mc:" << mc << " dp: " << dp.x << " " << dp.y;
	cerr << " dv: " << dv.x << " " << dv.y << endl;
#endif
    const double impact_norm = sqrt(impact*impact);
#ifdef DEBUG
    cerr << " giving impact " << impact.x << " " << impact.y << " with norm " << impact_norm << endl;
#endif
    if (impact_norm < 200.0) {
        impact *= 0.5 + 100.0 / impact_norm;
#ifdef DEBUG
        cerr << " adjusting norm, now impact is " << impact.x << " " << impact.y << endl;
#endif
	}
	v -= impact / m1;
#ifdef DEBUG
	cerr << "  new vel " << v.x << " " << v.y << " and " << other.v.x << " " << other.v.y << endl;
#endif
}

#ifdef DEBUG
const int debug_id1 = 3, debug_id2 = 13;
#endif

enum MoveType { NOOP, MOVE, THROW, ACCIO, FLIP, OBL, PETR };

class Move {
	public:
		MoveType type;
		int_fast16_t param1, param2, param3;
};

/*
class PlayerMove {
	public:
		Move ms[2];
};
*/

class ActiveSpell {
	public:
		MoveType stype;
		int_fast16_t wiz, param, rounds;
};

const Entity poles[4] = { 
	{{0, 1750}, {0, 0}, 300, INFINITY, 0, POLE, {}},
	{{0, 5750}, {0, 0}, 300, INFINITY, 0, POLE, {}},
	{{16000, 1750}, {0, 0}, 300, INFINITY, 0, POLE, {}},
	{{16000, 5750}, {0, 0}, 300, INFINITY, 0, POLE, {}}};

class GameState {
	public:
		Entity es[13];
		ActiveSpell sps[12];
		int_fast16_t mana[2], scores[2], round;
		int_fast16_t c_active, c_wizards, c_tsnaffles, c_asnaffles, c_bludgers, c_entities;
};

class Simulator {
	public:
		/*
		Entity es[17];
		ActiveSpell sps[12];
		int_fast16_t mana[2], scores[2];
		int_fast16_t c_active, c_entities, c_snaffles;
		int_fast16_t round;
		*/
		GameState w;

		//void step(const PlayerMove (&m)[2]);
		double score();
		void step(const vector<Move>& m);
		void _move_entity(int e_i, double dt);
		double _collide_time(int e_i, int e_j) const;
};

void Simulator::_move_entity(int e_i, double dt) {
#ifdef DEBUG
	if (e_i == debug_id1 || e_i == debug_id2) {
		cerr << "Moving entity " << e_i << " from " << w.es[e_i].p.x << " " << w.es[e_i].p.y << " with ";
		cerr << w.es[e_i].v.x << " " << w.es[e_i].v.y << endl;
	}
#endif
	w.es[e_i].move(dt);
#ifdef DEBUG
	if (e_i == debug_id1 || e_i == debug_id2) {
		cerr << "Resulting position " << w.es[e_i].p.x << " " << w.es[e_i].p.y << endl;
	}
#endif
}

double Simulator::_collide_time(int e_i, int e_j) const {
	double retval;
#ifdef DEBUG
	if (e_i == debug_id1 && e_j == debug_id2 || e_i == debug_id2 && e_j == debug_id1) {
		cerr << "Checking for collision between " << e_i << " and " << e_j << " " << endl;
		retval = w.es[e_i].sub_collide(w.es[e_j], true);
		cerr << " with result " << retval << endl;
	} else {
#endif
		retval = w.es[e_i].sub_collide(w.es[e_j], false);
#ifdef DEBUG
	}
#endif
	return retval;
}

double Simulator::score() {
	if (w.round >= 200 || w.scores[0] > w.c_asnaffles + w.scores[1] || w.scores[1] > w.c_asnaffles + w.scores[0]) {
		if (w.scores[0] > w.scores[1]) {
			return 2;
		} else if (w.scores[1] > w.scores[0]) {
			return 0;
		} else {
			return 1;
		}
	} else {
		return -1;
	}
}

void Simulator::step(const vector<Move>& m) {
	if (w.round >= 200 || w.scores[0] > w.c_asnaffles + w.scores[1] || w.scores[1] > w.c_asnaffles + w.scores[0]) return;

	//cerr << "Reading moves" << endl;
	for (int i = 0; i < w.c_wizards; ++i) {
		Entity& tw = w.es[i];
		const Move& tm = m[i];
		Point dir;

		switch (tm.type) {
			case MOVE:
				dir = Point { static_cast<double>(tm.param1), static_cast<double>(tm.param2) } - tw.p;
				tw.apply_force(dir, tm.param3);
				break;
			case THROW:
				if (tw.e.wi.snaffle != -1) {
					dir = Point { static_cast<double>(tm.param1), static_cast<double>(tm.param2) } - tw.p;
					//cerr << "Throwing snaffle, target " << tm.param1 << ", " << tm.param2 << endl;
					//cerr << "  direction " << dir.x << " " << dir.y << " old velocity ";
					//cerr << w.es[tw.e.wi.snaffle].v.x << " " << w.es[tw.e.wi.snaffle].v.y << endl;
					w.es[tw.e.wi.snaffle].apply_force(dir, tm.param3);
					//cerr << "  new velocity " << w.es[tw.e.wi.snaffle].v.x << " " << w.es[tw.e.wi.snaffle].v.y << endl;
				}
				break;
			default: break;
		}
	}

	// drop snaffles if not thrown
	//cerr << "Dropping snaffles" << endl;
	for (int i = 0; i < w.c_wizards; ++i) {
		w.es[i].e.wi.snaffle = -1;
	}
	for (int i = w.c_wizards; i < w.c_wizards + w.c_tsnaffles; ++i) {
		w.es[i].e.sn.carrier = -1;
	}

	// apply obliviate
	//cerr << "Applying active spells (obliviate)" << endl;
	for (int i = w.c_active - 1; i >= 0; --i) {
		if (w.sps[i].stype == OBL) {
			w.es[w.sps[i].param].e.bl.ignores |= (0x0001 << (w.es[w.sps[i].wiz].e.wi.team));
			w.sps[i].rounds--;
		}
	}

	// apply bludgers thrust
	//cerr << "Applying bludgers thrust" << endl;
	for (int i = w.c_wizards + w.c_tsnaffles; i < w.c_entities; ++i) {
		double min_dist = INFINITY;
		int_fast16_t w_idx = -1;
		for (int j = 0; j < w.c_wizards; ++j) {
			if (w.es[i].e.bl.last == j || (w.es[i].e.bl.ignores % 2 == 1 && w.es[j].e.wi.team == 0)
			                           || (w.es[i].e.bl.ignores & 0x0002 && w.es[j].e.wi.team == 1)) continue;
			const Point d_vec = w.es[j].p - w.es[i].p;
			double dist2 = d_vec * d_vec;
			if (dist2 < min_dist) {
				min_dist = dist2;
				w_idx = j;
			}
		}

		w.es[i].e.bl.ignores = 0;
		if (w_idx != -1) {
			//cerr << "Bludger " << i << " targeting wizard " << w_idx << endl;
			const Point d_vec = w.es[w_idx].p - w.es[i].p;
			w.es[i].apply_force(d_vec, 1000);
		}
	}
	
	// apply active spell forces
	//cerr << "Applying active spells" << endl;
	for (int i = w.c_active - 1; i >= 0; --i) {
		if (w.sps[i].stype == PETR) {
			w.es[w.sps[i].param].v.x = 0;
			w.es[w.sps[i].param].v.y = 0;
			w.sps[i].rounds = 0;
		}
	}
	
	for (int i = w.c_active - 1; i >= 0; --i) {
		if (w.sps[i].stype == ACCIO || w.sps[i].stype == FLIP) {
			if (w.es[w.sps[i].param].type == SNAFFLE && w.es[w.sps[i].param].e.sn.alive == 0) {
				w.sps[i].rounds = 0;
			} else {
				double effect_dir, base_power;
				if (w.sps[i].stype == ACCIO) {
					effect_dir = -1;
					base_power = 3000;
				} else {
					effect_dir = 1;
					base_power = 6000;
				}
				Point dist = w.es[w.sps[i].param].p - w.es[w.sps[i].wiz].p;
				//cerr << "Applying ";
				//if (w.sps[i].stype == ACCIO)
					//cerr << "ACCIO";
				//else
					//cerr << "FLIP";
				//cerr << " force to snaffle " << base_power << " " << dist*dist << " " << effect_dir << endl;
				//cerr << "-- by wizard " << w.sps[i].wiz << " at " << w.es[w.sps[i].wiz].p.x << " " << w.es[w.sps[i].wiz].p.y;
				//cerr << " to snaffle " << w.sps[i].param << " at " << w.es[w.sps[i].param].p.x << " " << w.es[w.sps[i].param].p.y << endl;
				//cerr << "-- current snaffle velocity " << w.es[w.sps[i].param].v.x << " " << w.es[w.sps[i].param].v.y << endl;
				if (dist*dist > 0) {
					w.es[w.sps[i].param].apply_force(dist, min(base_power / (dist*dist) * 1000 * 1000, 1000.0) * effect_dir);
					//cerr << "-- new snaffle velocity " << w.es[w.sps[i].param].v.x << " " << w.es[w.sps[i].param].v.y << endl;
				} else {
					if (w.sps[i].stype == ACCIO) {
						w.sps[i].rounds = 1; // will be decremented to 0
					}
				}
				w.sps[i].rounds--;
			}
		}

		if (w.sps[i].rounds == 0) {
			w.sps[i] = w.sps[w.c_active-1];
			w.c_active--;
		}
	}

	// movement and collisions
	//cerr << "Starting collision detection" << endl;
	double t = 0.0;
	while (t < 1.0) {
		double ct = 100.0;

		int e1=0, e2=0;
		for (int i = 0; i < w.c_entities; ++i) {
			if (w.es[i].type == SNAFFLE && (w.es[i].e.sn.alive == 0 || w.es[i].e.sn.carrier != -1)) continue;

			for (int j=i+1; j < w.c_entities; ++j) {
				if (w.es[j].type == SNAFFLE && (w.es[j].e.sn.alive == 0 || w.es[i].e.sn.carrier != -1)) continue;
				double rt = _collide_time(i, j);
				//double rt = es[i].sub_collide(es[j]);
				if (rt >= 0 && rt <= 1-t) {
					if (rt < ct || rt == ct &&
							(w.es[e1].type == WIZARD && e1 == i &&
							 w.es[e2].type == SNAFFLE && w.es[j].type == SNAFFLE &&
							 w.es[i].p.dist2(w.es[j].p) < w.es[e1].p.dist2(w.es[e2].p))) {
						ct = rt;
						e1 = i;
						e2 = j;
					}
				}
			}

			for (int j = 0; j < 4; ++j) {
				double rt = w.es[i].sub_collide(poles[j], false);
				if (rt >= 0 && rt <= 1-t && rt < ct) {
					ct = rt;
					e1 = i;
					e2 = w.c_entities + j;
				}
			}

			Entity& e = w.es[i];

			if (e.v.x > 0) {
				double tt = (16000.0 - e.r - e.p.x) / e.v.x;
				if (e.type == SNAFFLE) {
					const double npy = e.p.y + e.v.y * tt;
					if (1750.0 < npy && npy < 5750.0) {
						tt = (16000.0 - e.p.x) / e.v.x;
					}
				}
				if (tt >= 0 && tt < ct && tt <= 1-t) { 
					ct = tt;
					e1 = 0;
					e2 = -i-1;
				}
			}

			if (e.v.x < 0) {
				double tt = (e.r - e.p.x) / e.v.x;
				if (e.type == SNAFFLE) {
					const double npy = e.p.y + e.v.y * tt;
					if (1750.0 < npy && npy < 5750.0) {
						tt = (- e.p.x) / e.v.x;
					}
				}
				if (tt >= 0 && tt < ct && tt <= 1-t) {
					ct = tt;
					e1 = 2;
					e2 = -i-1;
				}
			}

			if (e.v.y > 0) {
				const double tt = (7500.0 - e.r - e.p.y) / e.v.y;
				if (tt >= 0 && tt < ct && tt <= 1-t) {
					ct = tt;
					e1 = 1;
					e2 = -i-1;
				}
			}

			if (e.v.y < 0) {
				const double tt = (e.r - e.p.y) / e.v.y;
				if (tt >= 0 && tt < ct && tt <= 1-t) {
					ct = tt;
					e1 = 3;
					e2 = -i-1;
				}
			}
		}

		if (ct <= 1) {
			//cerr << "-- skipping ahead by " << ct << " for collision " << e1 << " " << e2 << endl;
			for (int i = 0; i < w.c_entities; ++i) {
				_move_entity(i, ct);
				//es[i].move(ct);
			}
			if (e2 >= 0) {
				if (e2 >= w.c_entities) {
					w.es[e1].do_collide_pole(poles[e2 - w.c_entities]);
				} else {
					if (w.es[e1].type == WIZARD) {
						if (w.es[e2].type == SNAFFLE) {
							if (w.es[e1].e.wi.timeout == 0) {
								w.es[e1].e.wi.snaffle = e2;
								w.es[e1].e.wi.timeout = 3;
								//cerr << "Wizard " << e1 << " picks up snaffle " << e2 << endl;
								w.es[e2].e.sn.carrier = e1;
								w.es[e2].p = w.es[e1].p;  // FIXME remove pos/vel update, will be updated at end of turn anyways?
								w.es[e2].v = w.es[e1].v;
							} else {
								//cerr << "Wizard " << e1 << " wants to pick up snaffle " << e2 << " but is not ready yet" << endl;
							}
						} else if (w.es[e2].type == BLUDGER) {
							w.es[e2].e.bl.last = e1;
						}
					}
					if (w.es[e1].type != WIZARD || w.es[e2].type != SNAFFLE)
						w.es[e1].do_collide(w.es[e2]); //       specials:
															//       BLUDGER ignore wizard if hit
															//       SNAFFLE gets carried until end of round
				}
			} else {
				Entity& e = w.es[-e2-1];
				//cerr << "Boundary collision eX:" << e.p.x << " eY:" << e.p.y << " vX:" << e.v.x << " vY:" << e.v.y << endl;
				if (e1 % 2 == 0) { // X boundary wall
					if (e.type == SNAFFLE && e.p.y >= 1750 && e.p.y <= 5750) {
						w.scores[e1/2]++;
						e.e.sn.alive = 0;
						w.c_asnaffles--;
					} else {
						e.v.x *= -1;
					}
				} else { // Y boundary wall
					e.v.y *= -1;
				}
				//cerr << "after collision eX:" << e.p.x << " eY:" << e.p.y << " vX:" << e.v.x << " vY:" << e.v.y << endl;
			}
			t += ct;
		} else {
			for (int i = 0; i < w.c_entities; ++i) {
				_move_entity(i, 1-t);
				//es[i].move(1-t);
			}
			t = 1;
		}
	}

	// count down snaffle counter
	//cerr << "Counting down snaffle timeout" << endl;
	for (int i = 0; i < w.c_wizards; ++i) {
		if (w.es[i].e.wi.timeout > 0)  {
			//cerr << "Wizard " << i << " timer decreased from " << w.es[i].e.wi.timeout;
			w.es[i].e.wi.timeout--;
			//cerr << " to " << w.es[i].e.wi.timeout << endl;
		}
		w.es[i].end_turn();
	}

	// picked up snaffles move with wizards and end turn
	//cerr << "Adjusting held snaffle data" << endl;
	for (int i = w.c_wizards; i < w.c_entities; ++i) {
		if (w.es[i].type == SNAFFLE && w.es[i].e.sn.carrier != -1) {
			w.es[i].p = w.es[w.es[i].e.sn.carrier].p;
			w.es[i].v = w.es[w.es[i].e.sn.carrier].v;
		} else {
			w.es[i].end_turn();
		}
	}

	//cerr << "Reading spells" << endl;
	for (int i = 0; i < w.c_wizards; ++i) {
		Entity& tw = w.es[i];
		const Move& tm = m[i];
		Point dir;

		switch (tm.type) {
			case ACCIO:  // FIXME check for already active spells that should be canceled
				if (tm.param1 < w.c_wizards) {
					//cerr << "Skipping ACCIO of wizard " << i << " for wrong spell target" << endl;
					break; // ACCIO on WIZARD fails
				}
			case FLIP:
				if (w.mana[tw.e.wi.team] < 20 || (tm.param1 < w.c_wizards && w.es[tm.param1].e.wi.team == w.es[i].e.wi.team)) {
					//cerr << "Skipping spell of wizard " << i << " for not enough mana or wrong target" << endl;
					break; // FLIP on own WIZARD fails
				}
				w.mana[tw.e.wi.team] -= 20;
			case OBL:
				if (tm.type == OBL) {
					if (w.mana[tw.e.wi.team] < 5 || w.es[tm.param1].type != BLUDGER) {
						//cerr << "Skipping OBL of wizard " << i << " for not enough mana or wrong target" << endl;
						break;
					}
					w.mana[tw.e.wi.team] -= 5;
				}
				{
					int ns_id = w.c_active;
					for (int k = 0; k < w.c_active; ++k) {
						if (w.sps[k].stype == tm.type && w.sps[k].wiz == i) {
							//cerr << "Canceling previously active spell, and replacing it" << endl;
							ns_id = k;
							break;
						}
					}
					if (ns_id == w.c_active) {
						w.c_active++;
					}
					w.sps[ns_id].stype = tm.type; 
					w.sps[ns_id].wiz = i;
					w.sps[ns_id].param = tm.param1;
					w.sps[ns_id].rounds = (tm.type == ACCIO ? 6 : 3);
				}
				break;
			case PETR:
				if (w.mana[tw.e.wi.team] < 10 || (tm.param1 < w.c_wizards && w.es[tm.param1].e.wi.team == w.es[i].e.wi.team)) break;
				w.mana[tw.e.wi.team] -= 10;
				w.sps[w.c_active].stype = PETR;
				w.sps[w.c_active].wiz = i;
				w.sps[w.c_active].param = tm.param1;
				w.sps[w.c_active].rounds = 1;
				w.c_active++;
				//w.es[tm.param1].v.x = 0;
				//w.es[tm.param1].v.y = 0;
				break;
			default: break;
		}
	}


	if (w.mana[0] < 100) w.mana[0]++;
	if (w.mana[1] < 100) w.mana[1]++;

	w.round++;
	//cerr << "Step finished" << endl;
}
