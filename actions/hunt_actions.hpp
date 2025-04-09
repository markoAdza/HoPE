#ifndef HUNT_ACTIONS_HPP_INCLUDED
#define HUNT_ACTIONS_HPP_INCLUDED

#include "model/action_base.hpp"
#include "agents/agents.hpp"
//#include "agents/agents.hpp"
//#include "agents/pigeon.hpp"

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
				const auto sv = sim.sorted_view<Tag, pigeon_tag>(idx);

				if (sv.size())
				{ 
					const auto& target = sim.pop<pigeon_tag>()[sv[0].idx]; // nearest prey
					auto ofss = torus::ofs(Simulation::WH(), self->pos, target.pos);;

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


	}
}

#endif