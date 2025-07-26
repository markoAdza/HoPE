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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);
				const auto sv = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped, self->dir);

				if (sv.size())
				{
					const auto& target = sim.pop<pigeon_tag>()[sv[0].idx];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);

					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;

					int new_target = static_cast<int>(sv[0].idx);
					if (self->previous_target_i != new_target) {
						++self->target_switch_count;
					}

					self->previous_target_i = new_target;
					self->target_i = new_target;
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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);
				const auto sv = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped, self->dir);
				if (sv.size())
				{
					target_idx_ = sv[0].idx; // nearest prey
					self->target_i = static_cast<int>(target_idx_);
				}

				if (target_idx_ != -1)
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_]; // nearest prey
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);;
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;

					// confusability
					int new_target = static_cast<int>(target_idx_);
					if (self->previous_target_i != new_target) {
						++self->target_switch_count;
					}
					self->previous_target_i = new_target;
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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
				target_idx_ = static_cast<size_t>(-1);
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
						self->speed = prey_speed_scale_ * 15.0f;
					}
					else
					{
						glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);
						const auto sv = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped, self->dir);
						if (sv.size())
						{
							const auto& target = sim.pop<pigeon_tag>()[sv[0].idx]; // nearest prey
							target_idx_ = sv[0].idx; // nearest prey

							//confusability
							int new_target = static_cast<int>(sv[0].idx);
							if (self->previous_target_i != new_target) {
								++self->target_switch_count;
							}

							self->previous_target_i = new_target;
							self->target_i = new_target;
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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);

				auto vis = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped, self->dir);

				std::vector<size_t> candidates;
				candidates.reserve(vis.size());
				for (size_t vi = 0; vi < vis.size(); ++vi) {
					size_t prey_idx = vis[vi].idx;
					if (sim.flock_of<pigeon_tag>(prey_idx) == self->target_f) {
						candidates.push_back(prey_idx);
					}
				}

				float best_isolation = -1.0f;
				size_t most_isolated_idx = static_cast<size_t>(-1);

				const auto& prey_pop = sim.pop<pigeon_tag>();
				for (size_t prey_idx : candidates) {
					auto nb = sim.sorted_view<pigeon_tag, pigeon_tag>(prey_idx);

					float distance_sum = 0.0f;
					for (size_t k = 0; k < std::min(num_neighbors_, nb.size()); ++k) {
						size_t neighbor_idx = nb[k].idx;
						if (!sim.is_alive<pigeon_tag>(neighbor_idx)) continue; // skip dead neighbor
						glm::vec2 diff = torus::ofs(
							Simulation::WH(),
							prey_pop[prey_idx].pos,
							prey_pop[neighbor_idx].pos
						);
						distance_sum += glm::length(diff);
					}

					if (distance_sum > best_isolation) {
						best_isolation = distance_sum;
						most_isolated_idx = prey_idx;
					}
				}

				target_idx_ = most_isolated_idx;

				if (target_idx_ != static_cast<size_t>(-1) &&
					sim.is_alive<pigeon_tag>(target_idx_))
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;

					//confusability
					int new_target = static_cast<int>(target_idx_);
					if (self->previous_target_i != new_target){
						++self->target_switch_count;
					}
					self->previous_target_i = new_target;
					self->target_i = new_target;
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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				const auto& prey_pop = sim.pop<pigeon_tag>();
				glm::vec2 centroid = sim.compute_flock_centroid<pigeon_tag>(self->target_f);

				glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);
				auto vis = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped,self->dir);

				float max_dist_to_centroid = -1.0f;
				size_t most_peripheral_idx = static_cast<size_t>(-1);

				for (size_t vi = 0; vi < vis.size(); ++vi)
				{
					size_t prey_idx = vis[vi].idx;

					if (sim.flock_of<pigeon_tag>(prey_idx) != self->target_f)
						continue;

					glm::vec2 from_cent = torus::ofs(Simulation::WH(), centroid, prey_pop[prey_idx].pos);
					float dist_to_cent = glm::length(from_cent);

					if (dist_to_cent > max_dist_to_centroid)
					{
						max_dist_to_centroid = dist_to_cent;
						most_peripheral_idx = prey_idx;
					}
				}

				target_idx_ = most_peripheral_idx;

				if (target_idx_ != static_cast<size_t>(-1) && sim.is_alive<pigeon_tag>(target_idx_))
				{
					const auto& target = sim.pop<pigeon_tag>()[target_idx_];
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);
					const auto Fdir = math::save_normalize(ofss, vec_t(0.f)) * w_;
					self->steering += Fdir;
					self->speed = prey_speed_scale_ * target.speed;

					//confusability
					int new_target = static_cast<int>(target_idx_);
					if (self->previous_target_i != new_target){
						++self->target_switch_count;
					}
					self->previous_target_i = new_target;
					self->target_i = new_target;
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
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
			{
				glm::vec2 pred_pos_wrapped = torus::wrap(Simulation::WH(), self->pos);
				auto vis = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, pred_pos_wrapped, self->dir);

				std::vector<size_t> valid_prey_indices;
				valid_prey_indices.reserve(vis.size());

				for (size_t i = 0; i < vis.size(); ++i) {
					size_t prey_idx = vis[i].idx;
					if (sim.flock_of<pigeon_tag>(prey_idx) != self->target_f)
						continue;
					valid_prey_indices.push_back(prey_idx);
				}

				if (!valid_prey_indices.empty()) {
					std::uniform_int_distribution<size_t> dist(0, valid_prey_indices.size() - 1);
					size_t choice = valid_prey_indices[dist(thread_local_rng())];
					target_idx_ = choice;

					//confusability
					int new_target = static_cast<int>(target_idx_);
					if (self->previous_target_i != new_target) {
						++self->target_switch_count;
					}

					self->previous_target_i = new_target;
					self->target_i = static_cast<int>(target_idx_);
				}

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
			void on_entry(agent_type* self, size_t, tick_t, const Simulation&) {
				self->previous_target_i = -1;
				self->target_switch_count = 0;
				self->num_catches = 0;
				self->last_caught_prey_id = -1;
				target_idx_ = size_t(-1);
			}

			void check_state_exit(const tick_t& state_dur, tick_t& state_exit_t)
			{
			}

			void operator()(agent_type* self, size_t idx, tick_t, const Simulation& sim) {
				// Find most isolated prey among visible neighbors
				glm::vec2 wrapped = torus::wrap(Simulation::WH(), self->pos);
				auto vis = sim.sorted_view_with_occlusion<pred_tag, pigeon_tag>(idx, wrapped, self->dir);
				std::vector<size_t> candidates;
				for (auto& v : vis) {
					if (sim.flock_of<pigeon_tag>(v.idx) == self->target_f)
						candidates.push_back(v.idx);
				}
				float best_iso = -1.0f;
				size_t isolate_idx = size_t(-1);
				const auto& pop = sim.pop<pigeon_tag>();
				for (auto pi : candidates) {
					auto nb = sim.sorted_view<pigeon_tag, pigeon_tag>(pi);
					float sumd = 0.0f;
					for (size_t k = 0; k < std::min(num_neighbors_, nb.size()); ++k) {
						if (!sim.is_alive<pigeon_tag>(nb[k].idx)) continue;
						auto d = glm::length(torus::ofs(Simulation::WH(), pop[pi].pos, pop[nb[k].idx].pos));
						sumd += d;
					}
					if (sumd > best_iso) { best_iso = sumd; isolate_idx = pi; }
				}
				if (isolate_idx == size_t(-1) || !sim.is_alive<pigeon_tag>(isolate_idx)) {
					// no valid target
					return;
				}

				// Compute offset to target
				const auto& prey = pop[isolate_idx];
				auto ofs = torus::ofs(Simulation::WH(), self->pos, prey.pos);
				auto dir = math::save_normalize(ofs, vec_t(0.0f));
				float dist = glm::length(ofs);

				constexpr size_t max_steer_dist_ = 10;
				const float ramp = glm::clamp((max_steer_dist_ - dist) / max_steer_dist_, 0.0f, 1.0f);
				float gain = steering_factor_ * (1.0f + ramp);
				self->steering = dir * gain;
				self->speed = stoop_speed_factor_ * prey.speed;
				
				// track switches
				int new_i = int(isolate_idx);
				if (self->previous_target_i != -1 && self->previous_target_i != new_i)
					++self->target_switch_count;
				self->previous_target_i = new_i;
				self->target_i = new_i;
			}
		private:
			float w_ = 0, prey_speed_scale_ = 1.0f;
			float stoop_distance_threshold_ = 0;
			float stoop_speed_factor_ = 1.0f;
			float steering_factor_ = 0.1f;
			size_t num_neighbors_ = 5;
			size_t target_idx_;
		};

	}
}

#endif