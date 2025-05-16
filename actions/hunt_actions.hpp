#ifndef HUNT_ACTIONS_HPP_INCLUDED
#define HUNT_ACTIONS_HPP_INCLUDED

#include "model/action_base.hpp"
#include "agents/agents.hpp"
//#include "agents/agents.hpp"
//#include "agents/pigeon.hpp"

#include <random>
#include <thread>

inline std::mt19937& thread_local_rng() {
	thread_local std::mt19937 rng(std::random_device{}());
	return rng;
}


namespace model {
	namespace actions {

		template <typename Agent>
		class chase_closest_prey
		{ // go towards the closest individual from the flock // PIGEON SPECIFIC (pigeons tag in)

			make_action_from_this(chase_closest_prey);

		public:
			chase_closest_prey() {}
			chase_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
				prey_speed_scale_ = J["prey_speed_scale"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto sv = sim.sorted_view_with_occlusion<Tag, pigeon_tag>(idx, self->pos, self->dir);

				if (sv.size())
				{
					const auto& target = sim.pop<pigeon_tag>()[sv[0].idx];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);

					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
					self->target_i = static_cast<int>(sv[0].idx);
				}
			}

		private:
			float w_ = 0;           // [1]
			float prey_speed_scale_ = 0; // speed in relation to the preys speed [1]
		};


		template <typename Agent>
		class lock_on_closest_prey
		{ // lock on to the closest individual at entry and go towards it for the rest of the action // PIGEON SPECIFIC (pigeons tag in)

			make_action_from_this(lock_on_closest_prey);

		public:
			lock_on_closest_prey() {}
			lock_on_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
				prey_speed_scale_ = J["prey_speed_scale"];                       // [1]
				target_idx_ = -1;
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto sv = sim.sorted_view<Tag, pigeon_tag>(idx);
				if (sv.size())
				{
					target_idx_ = sv[0].idx; // nearest prey
					self->target_i = static_cast<int>(target_idx_);
				}
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (target_idx_ != -1)
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_]; // nearest prey
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);;
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;           // [1]
			float prey_speed_scale_ = 0; // speed in relation to the preys speed [1]
			size_t target_idx_; //
		};

		template <typename Agent>
		class lock_on_centroid_prey
		{
			make_action_from_this(lock_on_centroid_prey);

		public:
			lock_on_centroid_prey() {}
			lock_on_centroid_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
				prey_speed_scale_ = J["prey_speed_scale"]; // [1]
				target_idx_ = -1;
				centroid_threshold_ = J["centroid_threshold"]; // Distance threshold to switch to closest prey
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (target_idx_ == -1)
				{

					// Compute the centroid of the flock the predator is targeting
					centroid_ = sim.compute_flock_centroid<pigeon_tag>(self->target_f);
					float dist_to_centroid = glm::distance(self->pos, centroid_);

					if (dist_to_centroid > centroid_threshold_)
					{
						auto ofss = torus::ofs(Simulation::WH(), self->pos, centroid_);
						const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
						self->steering += Fdir;
						self->speed = prey_speed_scale_ * 10.0f;  // Adjust speed as needed
					}
					else
					{
						const auto sv = sim.sorted_view<Tag, pigeon_tag>(idx);
						if (sv.size())
						{
							const auto& target = sim.pop<pigeon_tag>()[sv[0].idx]; // nearest prey
							target_idx_ = sv[0].idx; // nearest prey
							self->target_i = static_cast<int>(target_idx_);
						}
					}

				}else if(target_idx_ != -1)
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;                       // [1]
			float prey_speed_scale_ = 0;        // Speed in relation to the prey's speed [1]
			size_t target_idx_;
			float centroid_threshold_ = 10.0f;  // The distance threshold to switch to the closest prey
			glm::vec2 centroid_;                // The centroid of the flock
		};


		template <typename Agent>
		class avoid_closest_prey
		{ // go towards the closest individual from the flock // PIGEON SPECIFIC (pigeons tag in)

			make_action_from_this(avoid_closest_prey);

		public:
			avoid_closest_prey() {}
			avoid_closest_prey(size_t, const json& J)
			{
				w_ = J["w"];                       // [1]
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto sv = sim.sorted_view<Tag, pigeon_tag>(idx);

				if (sv.size())
				{
					const auto& flock_ind = sim.pop<pigeon_tag>()[sv[0].idx]; // nearest prey
					auto ofss = torus::ofs(Simulation::WH(), flock_ind.pos, self->pos);;

					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
				}
			}

		public:
			float w_ = 0;           // [1]
		};

		template <typename Agent>
		class hunt_most_isolated_prey
		{
			make_action_from_this(hunt_most_isolated_prey);

		public:
			hunt_most_isolated_prey() {}

			hunt_most_isolated_prey(size_t, const json& J)
			{
				w_ = J["w"];
				prey_speed_scale_ = J["prey_speed_scale"];
				num_neighbors_ = J["num_neighbors"];
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto& prey_pop = sim.pop<pigeon_tag>();
				float max_distance_sum = -1.0f;
				size_t most_isolated_idx = static_cast<size_t>(-1);

				for (size_t i = 0; i < prey_pop.size(); ++i)
				{
					if (!sim.is_alive<pigeon_tag>(i)) continue;

					const auto neighbors = sim.sorted_view<pigeon_tag, pigeon_tag>(i); // prey's own neighbors
					float distance_sum = 0.0f;

					for (size_t j = 0; j < std::min(num_neighbors_, neighbors.size()); ++j)
					{
						const auto& neighbor = prey_pop[neighbors[j].idx];
						distance_sum += glm::length(torus::ofs(Simulation::WH(), prey_pop[i].pos, neighbor.pos));
					}

					if (distance_sum > max_distance_sum)
					{
						max_distance_sum = distance_sum;
						most_isolated_idx = i;
					}
				}

				target_idx_ = most_isolated_idx;
				self->target_i = static_cast<int>(target_idx_);
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (target_idx_ != static_cast<size_t>(-1) && sim.is_alive<pigeon_tag>(target_idx_))
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;
			float prey_speed_scale_ = 0;
			size_t target_idx_ = static_cast<size_t>(-1);
			size_t num_neighbors_ = 5;  // Number of neighbors to evaluate isolation
		};

		template <typename Agent>
		class hunt_most_peripheral_prey
		{
			make_action_from_this(hunt_most_peripheral_prey);

		public:
			hunt_most_peripheral_prey() {}

			hunt_most_peripheral_prey(size_t, const json& J)
			{
				w_ = J["w"];
				prey_speed_scale_ = J["prey_speed_scale"];
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto& prey_pop = sim.pop<pigeon_tag>();
				float max_dist_to_centroid = -1.0f;
				size_t most_peripheral_idx = static_cast<size_t>(-1);

				for (size_t i = 0; i < prey_pop.size(); ++i)
				{
					if (!sim.is_alive<pigeon_tag>(i)) continue;

					// ONLY consider prey from the targeted flock
					if (sim.flock_of<pigeon_tag>(i) != self->target_f) continue;

					glm::vec2 centroid = sim.compute_flock_centroid<pigeon_tag>(self->target_f);

					float dist = glm::length(torus::ofs(Simulation::WH(), prey_pop[i].pos, centroid));

					if (dist > max_dist_to_centroid)
					{
						max_dist_to_centroid = dist;
						most_peripheral_idx = i;
					}
				}

				target_idx_ = most_peripheral_idx;
				self->target_i = static_cast<int>(target_idx_);
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (target_idx_ != static_cast<size_t>(-1) && sim.is_alive<pigeon_tag>(target_idx_))
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;
			float prey_speed_scale_ = 0;
			size_t target_idx_ = static_cast<size_t>(-1);
		};

		template <typename Agent>
		class hunt_random_prey
		{
			make_action_from_this(hunt_random_prey);

		public:
			hunt_random_prey() {}

			hunt_random_prey(size_t, const json& J)
			{
				w_ = J["w"];
				prey_speed_scale_ = J["prey_speed_scale"];
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto& prey_pop = sim.pop<pigeon_tag>();
				std::vector<size_t> valid_prey_indices;

				for (size_t i = 0; i < prey_pop.size(); ++i)
				{
					if (!sim.is_alive<pigeon_tag>(i)) continue;

					// ONLY consider prey from the targeted flock
					if (sim.flock_of<pigeon_tag>(i) != self->target_f) continue;

					valid_prey_indices.push_back(i);
				}

				if (!valid_prey_indices.empty())
				{
					std::uniform_int_distribution<size_t> dist(0, valid_prey_indices.size() - 1);
					const size_t rand_idx = valid_prey_indices[dist(thread_local_rng())];
					target_idx_ = rand_idx;
					self->target_i = static_cast<int>(target_idx_);
				}
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				if (target_idx_ != static_cast<size_t>(-1) && sim.is_alive<pigeon_tag>(target_idx_))
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;
				}
			}

		private:
			float w_ = 0;
			float prey_speed_scale_ = 0;
			size_t target_idx_ = static_cast<size_t>(-1);
		};

		template <typename Agent>
		class stooping_hunt {
			make_action_from_this(stooping_hunt);

		public:
			stooping_hunt() {}

			stooping_hunt(size_t, const json& J) {
				w_ = J["w"];
				prey_speed_scale_ = J["prey_speed_scale"];
				stoop_distance_threshold_ = J["stoop_distance_threshold"];
				stoop_speed_factor_ = J["stoop_speed_factor"];
				steering_factor_ = J["steering_factor"];
				num_neighbors_ = J["num_neighbors"];
			}

			void on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) {
				stooping_ = false;
				target_idx_ = static_cast<size_t>(-1);
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim) {
				if (!stooping_) {
					try_initiate_stoop(self, sim);
				}
				else {
					execute_stoop(self, sim);
				}
			}

		private:
			float w_ = 0;
			float prey_speed_scale_ = 1.0f;
			float stoop_distance_threshold_ = 50.0f;
			float stoop_speed_factor_ = 2.0f;
			float steering_factor_ = 0.05f;
			size_t num_neighbors_ = 5;

			bool stooping_ = false;
			size_t target_idx_ = static_cast<size_t>(-1);
			vec_t locked_dir_;

			void try_initiate_stoop(agent_type* self, const Simulation& sim) {
				const vec_t predator_pos = self->pos;
				const vec_t flock_centroid = sim.compute_flock_centroid<pigeon_tag>(self->target_f);
				float dist = glm::length(torus::ofs(Simulation::WH(), predator_pos, flock_centroid));

				if (dist > stoop_distance_threshold_) return;

				const auto& prey_pop = sim.pop<pigeon_tag>();
				float max_isolation = -1.0f;

				for (size_t i = 0; i < prey_pop.size(); ++i) {
					if (!sim.is_alive<pigeon_tag>(i)) continue;
					if (sim.flock_of<pigeon_tag>(i) != self->target_f) continue;

					auto neighbors = sim.sorted_view<pigeon_tag, pigeon_tag>(i);
					float distance_sum = 0.0f;

					for (size_t j = 0; j < std::min(num_neighbors_, neighbors.size()); ++j)
						distance_sum += glm::length(torus::ofs(Simulation::WH(), prey_pop[i].pos, prey_pop[neighbors[j].idx].pos));

					if (distance_sum > max_isolation) {
						max_isolation = distance_sum;
						target_idx_ = i;
					}
				}

				if (target_idx_ != static_cast<size_t>(-1)) {
					const auto& target = prey_pop[target_idx_];
					vec_t offset = torus::ofs(Simulation::WH(), self->pos, target.pos);
					locked_dir_ = math::save_normalize(offset, vec_t(0.f));
					stooping_ = true;
					self->target_i = static_cast<int>(target_idx_);
				}
			}

			void execute_stoop(agent_type* self, const Simulation& sim) {
				if (!sim.is_alive<pigeon_tag>(target_idx_)) {
					stooping_ = false;
					target_idx_ = static_cast<size_t>(-1);
					return;
				}

				const auto& target = sim.pop<pigeon_tag>()[target_idx_];

				const vec_t to_target = torus::ofs(Simulation::WH(), self->pos, target.pos);
				const float distance = glm::length(to_target);

				const vec_t desired_dir = math::save_normalize(to_target, locked_dir_);
				const float closeness = glm::clamp(1.0f - distance / stoop_distance_threshold_, 0.0f, 1.0f);

				// Ensure some minimum steering remains
				const float min_steering = 0.2f;
				const float adaptive_steering = glm::mix(steering_factor_, min_steering, closeness);

				const vec_t steering_dir = glm::mix(locked_dir_, desired_dir, adaptive_steering);
				self->steering += steering_dir * w_;
				self->speed = target.speed * stoop_speed_factor_;
			}
		};

	}
}

#endif